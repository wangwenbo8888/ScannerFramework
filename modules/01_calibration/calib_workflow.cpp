#include "calib_workflow.h"

#include "intrinsic_calib_cpu.h"
#include "extrinsic_calib_cpu.h"
#include "stereo_rectify_cpu.h"
#include "stereo_rectify_temp_table_cpu.h"
#include "intrinsic_compensate_cpu.h"
#include "extrinsic_compensate_cpu.h"

// 阶段2 姿态判断算子
#include "mask_extract_cuda.h"
#include "frame_filter_cuda.h"
#include "region_analyze_cuda.h"
#include "image_split_cpu.h"
#include "zernike_edge_cpu.h"
#include "image_merge_cpu.h"
#include "undistort_points_cpu.h"
#include "ellipse_fit_cpu.h"
#include "marker_match_cpu.h"
#include "epipolar_intersect_cpu.h"
#include "edge_match_cpu.h"
#include "point_reconstruct_cpu.h"
#include "frame_fuse_cpu.h"
#include "pose_estimate_cpu.h"

// 阶段4 激光标定算子
#include "laser_label_cuda.h"
#include "steger_extract_cuda.h"
#include "undistort_points_cuda.h"
#include "epipolar_interp_cuda.h"
#include "laser_match_cuda.h"
#include "laser_reconstruct_cuda.h"
#include "projector_joint_calib.h"

#include <opencv2/cudaimgproc.hpp>
#include <spdlog/spdlog.h>

#include <nlohmann/json.hpp>
#include <fstream>

namespace calibration {

// ============================================================================
// 辅助：生成棋盘格世界坐标
// ============================================================================
static std::vector<cv::Point3f> generateObjectPoints()
{
    std::vector<cv::Point3f> pts;
    for (int r = 0; r < CHESSBOARD_ROWS; ++r)
        for (int c = 0; c < CHESSBOARD_COLS; ++c)
            pts.emplace_back(
                static_cast<float>(c * CHESSBOARD_SQUARE_MM),
                static_cast<float>(r * CHESSBOARD_SQUARE_MM),
                0.0f);
    return pts;
}

// ============================================================================
// 完整相机标定流程（内参→外参→矫正）
// 输入：采集阶段已检测的角点
// ============================================================================
CameraCalibResult runCameraCalibration(
    const CameraCalibInput& input,
    std::function<void(int, const std::string&)> progress)
{
    CameraCalibResult result;
    auto report = [&](int pct, const std::string& step) {
        if (progress) progress(pct, step);
    };

    // 角点数量检查
    int n = std::min(input.leftCorners.size(), input.rightCorners.size());
    if (n < 5) {
        result.message = "valid frames < 5: " + std::to_string(n);
        return result;
    }
    result.validFrameCount = n;

    cv::Size patternSize(CHESSBOARD_COLS, CHESSBOARD_ROWS);
    cv::Size imageSize(input.imageWidth, input.imageHeight);
    auto objectPoints = generateObjectPoints();

    // ===== 1. 内参标定 =====
    report(10, "intrinsic calibration...");

    calib::IntrinsicCalibParams intrinsicParams;
    intrinsicParams.chessboard_width = CHESSBOARD_COLS;
    intrinsicParams.chessboard_height = CHESSBOARD_ROWS;
    intrinsicParams.square_size_mm = CHESSBOARD_SQUARE_MM;
    intrinsicParams.image_width = input.imageWidth;
    intrinsicParams.image_height = input.imageHeight;

    calib::IntrinsicCalibCPU intrinsicCalib(intrinsicParams);
    calib::IntrinsicCalibResult intrinsicResult;

    if (!intrinsicCalib.Execute(input.leftCorners, input.rightCorners, intrinsicResult) ||
        !intrinsicResult.success)
    {
        result.message = "intrinsic failed: " + intrinsicResult.message;
        return result;
    }

    result.cameraMatrixL = intrinsicResult.left.camera_matrix.clone();
    result.distCoeffsL = intrinsicResult.left.dist_coeffs.clone();
    result.cameraMatrixR = intrinsicResult.right.camera_matrix.clone();
    result.distCoeffsR = intrinsicResult.right.dist_coeffs.clone();
    result.intrinsicRMS = intrinsicResult.reproj_error_mean;

    report(40, "intrinsic done, RMS=" + std::to_string(result.intrinsicRMS));

    // ===== 2. 外参标定 =====
    report(45, "extrinsic calibration...");

    calib::ExtrinsicCalibCpuParams extrinsicParams;
    extrinsicParams.leftPointsPerView = input.leftCorners;
    extrinsicParams.rightPointsPerView = input.rightCorners;
    extrinsicParams.objectPoints = objectPoints;
    extrinsicParams.imageSize = imageSize;
    extrinsicParams.patternSize = patternSize;
    extrinsicParams.squareSize = static_cast<float>(CHESSBOARD_SQUARE_MM);

    calib::ExtrinsicCalibCpu extrinsicCalib(extrinsicParams);
    auto extrinsicResult = extrinsicCalib.Execute(
        result.cameraMatrixL, result.distCoeffsL,
        result.cameraMatrixR, result.distCoeffsR);

    if (!extrinsicResult.success) {
        result.message = "extrinsic failed: " + extrinsicResult.message;
        return result;
    }

    result.R = extrinsicResult.R.clone();
    result.T = extrinsicResult.T.clone();
    result.E = extrinsicResult.E.clone();
    result.F = extrinsicResult.F.clone();
    result.stereoReprojError = extrinsicResult.stereoReprojError;
    result.epipolarErrorMean = extrinsicResult.epipolarErrorMean;

    report(70, "extrinsic done");

    // ===== 3. 立体矫正 =====
    report(75, "stereo rectify...");

    calib::StereoRectifyCpuParams rectifyParams;
    rectifyParams.cameraMatrixL = result.cameraMatrixL;
    rectifyParams.distCoeffsL = result.distCoeffsL;
    rectifyParams.cameraMatrixR = result.cameraMatrixR;
    rectifyParams.distCoeffsR = result.distCoeffsR;
    rectifyParams.imageSize = imageSize;
    rectifyParams.R = result.R;
    rectifyParams.T = result.T;

    calib::StereoRectifyCpu rectify(rectifyParams);
    auto rectifyResult = rectify.Execute();

    if (!rectifyResult.success) {
        result.message = "rectify failed: " + rectifyResult.message;
        return result;
    }

    result.R1 = rectifyResult.R1.clone();
    result.R2 = rectifyResult.R2.clone();
    result.P1 = rectifyResult.P1.clone();
    result.P2 = rectifyResult.P2.clone();
    result.Q = rectifyResult.Q.clone();
    result.validRoiL = rectifyResult.validRoiLeft;
    result.validRoiR = rectifyResult.validRoiRight;

    report(95, "rectify done");

    // ===== 4. 温度补偿表（可选）=====
    if (input.cte > 0 && input.tempRangeMax > input.tempRangeMin) {
        report(96, "temperature compensation tables...");

        // 4a. 内参补偿表 L/R
        calib::CameraIntrinsics cL{
            result.cameraMatrixL.at<double>(0, 0),
            result.cameraMatrixL.at<double>(1, 1),
            result.cameraMatrixL.at<double>(0, 2),
            result.cameraMatrixL.at<double>(1, 2),
            input.referenceTemp
        };
        calib::CameraIntrinsics cR{
            result.cameraMatrixR.at<double>(0, 0),
            result.cameraMatrixR.at<double>(1, 1),
            result.cameraMatrixR.at<double>(0, 2),
            result.cameraMatrixR.at<double>(1, 2),
            input.referenceTemp
        };
        calib::IntrinsicCompensateCPUParams icp;
        icp.cte = input.cte;
        icp.tempStep = input.tempStep;
        icp.tempRangeMin = input.tempRangeMin;
        icp.tempRangeMax = input.tempRangeMax;
        calib::IntrinsicCompensateCPU icomp(icp);
        auto tableIntrinL = icomp.Execute(cL);
        auto tableIntrinR = icomp.Execute(cR);

        // 4b. 外参补偿表
        calib::CameraExtrinsics ce;
        ce.referenceTemp = input.referenceTemp;
        for (int i = 0; i < 3; ++i)
            ce.T[i] = result.T.at<double>(i);
        for (int i = 0; i < 9; ++i)
            ce.R[i] = result.R.at<double>(i / 3, i % 3);
        calib::ExtrinsicCompensateCPUParams ecp;
        ecp.cte = input.cte;
        ecp.tempStep = input.tempStep;
        ecp.tempRangeMin = input.tempRangeMin;
        ecp.tempRangeMax = input.tempRangeMax;
        calib::ExtrinsicCompensateCPU ecomp(ecp);
        auto tableExtrin = ecomp.Execute(ce);

        // 4c. 立体矫正温度补偿表
        calib::StereoRectifyTempTableParams strp;
        strp.cameraMatrixL = result.cameraMatrixL;
        strp.distCoeffsL = result.distCoeffsL;
        strp.cameraMatrixR = result.cameraMatrixR;
        strp.distCoeffsR = result.distCoeffsR;
        strp.imageSize = cv::Size(input.imageWidth, input.imageHeight);
        strp.R = result.R;
        strp.T = result.T;
        strp.referenceTemp = input.referenceTemp;
        strp.cte = input.cte;
        strp.tempStep = input.tempStep;
        strp.tempRangeMin = input.tempRangeMin;
        strp.tempRangeMax = input.tempRangeMax;
        strp.alpha = input.rectifyAlpha;
        strp.flags = input.rectifyFlags;
        calib::StereoRectifyTempTableCpu strtab(strp);
        auto tableSR = strtab.Execute();

        // 序列化为 JSON
        nlohmann::json tempJson;
        tempJson["referenceTemp"] = input.referenceTemp;
        tempJson["cte"] = input.cte;
        tempJson["intrinsicL"] = tableIntrinL.toJson();
        tempJson["intrinsicR"] = tableIntrinR.toJson();
        tempJson["extrinsic"] = tableExtrin.toJson();
        tempJson["stereoRectify"]["success"] = tableSR.success;
        tempJson["stereoRectify"]["tableSize"] = tableSR.table.size();
        result.tempTablesJson = tempJson.dump();
        result.hasTempTables = true;

        report(99, "temperature tables done");
    }

    report(100, "calibration done");
    result.success = true;
    return result;
}

// ============================================================================
// 阶段2 姿态判断（单帧标记点处理链，14算子）
// ============================================================================
PoseEstimationResult runPoseEstimation(
    const PoseEstimationInput& input,
    const PoseEstimationResult* prevFrame,
    std::function<void(int, const std::string&)> progress)
{
    PoseEstimationResult result;
    auto report = [&](int pct, const std::string& step) {
        if (progress) progress(pct, step);
    };

    if (input.leftMarkerGray.empty() || input.rightMarkerGray.empty()) {
        result.message = "empty input images";
        return result;
    }

    cv::cuda::Stream stream;

    // ===== 2-1 mask_extract (L/R) =====
    report(5, "mask_extract...");
    calib::MaskExtractCUDA maskOp;
    auto maskL = maskOp.Execute(input.leftMarkerGray, stream);
    auto maskR = maskOp.Execute(input.rightMarkerGray, stream);
    if (!maskL.success || !maskR.success) {
        result.message = "mask_extract failed";
        maskOp.Destroy();
        return result;
    }

    // ===== 2-1b frame_filter (L/R) =====
    report(10, "frame_filter...");
    calib::FrameFilterCUDA filterOp;
    auto filterL = filterOp.Execute(*maskL.d_cleanedMask, stream);
    auto filterR = filterOp.Execute(*maskR.d_cleanedMask, stream);
    if (!filterL.isMarkerFrame || !filterR.isMarkerFrame) {
        result.message = "not marker frame (laser frame filtered)";
        maskOp.Destroy();
        filterOp.Destroy();
        return result;
    }

    // ===== 2-2 CCL (L/R) =====
    report(15, "CCL...");
    calib::RegionAnalyzerCUDA cclOp;
    auto cclL = cclOp.Execute(*maskL.d_cleanedMask, stream);
    auto cclR = cclOp.Execute(*maskR.d_cleanedMask, stream);
    stream.waitForCompletion();
    if (!cclL.success || !cclR.success || cclL.components.empty()) {
        result.message = "CCL failed or empty";
        maskOp.Destroy();
        filterOp.Destroy();
        cclOp.Destroy();
        return result;
    }

    // CCL → roiRects (用于 image_split)
    auto toRects = [](const calib::RegionAnalysisResult& res) {
        std::vector<cv::Rect> rects;
        for (const auto& c : res.components)
            rects.emplace_back(c.boundingBoxX, c.boundingBoxY, c.boundingBoxWidth, c.boundingBoxHeight);
        return rects;
    };
    auto roiRectsL = toRects(cclL);
    auto roiRectsR = toRects(cclR);

    maskOp.Destroy();
    filterOp.Destroy();
    cclOp.Destroy();

    // ===== 2-3 image_split (L/R) =====
    report(25, "image_split...");
    calib::ImageSplitCPU splitOp;
    auto splitL = splitOp.Execute(input.leftMarkerGray, roiRectsL);
    auto splitR = splitOp.Execute(input.rightMarkerGray, roiRectsR);
    if (!splitL.success || splitL.splitImages.empty()) {
        result.message = "image_split failed";
        return result;
    }

    // ===== 2-4 zernike_edge (per sub-image, L/R) =====
    report(35, "zernike_edge...");
    calib::ZernikeEdgeCPU zernikeOp;
    std::vector<std::vector<calib::EdgePoint>> edgePointsPerSubL, edgePointsPerSubR;
    for (const auto& sub : splitL.splitImages) {
        auto er = zernikeOp.Execute(sub);
        edgePointsPerSubL.push_back(er.edgePoints);
    }
    for (const auto& sub : splitR.splitImages) {
        auto er = zernikeOp.Execute(sub);
        edgePointsPerSubR.push_back(er.edgePoints);
    }

    // ===== 2-5 image_merge (L/R) =====
    report(45, "image_merge...");
    calib::ImageMergeCPU mergeOp;
    auto mergeL = mergeOp.Execute(edgePointsPerSubL, roiRectsL);
    auto mergeR = mergeOp.Execute(edgePointsPerSubR, roiRectsR);
    if (!mergeL.success || mergeL.mergedEdgePoints.empty()) {
        result.message = "image_merge failed or empty";
        return result;
    }

    // ===== 2-6 undistort (L/R) =====
    report(55, "undistort...");
    calib::MarkerUndistortCPU undistortOp;
    calib::MarkerUndistortCPUParams undistortParams;

    // 从 cameraMatrix 填充 cam1(L) 和 cam2(R) 参数
    undistortParams.fx1 = input.cameraMatrixL.at<double>(0, 0);
    undistortParams.fy1 = input.cameraMatrixL.at<double>(1, 1);
    undistortParams.cx1 = input.cameraMatrixL.at<double>(0, 2);
    undistortParams.cy1 = input.cameraMatrixL.at<double>(1, 2);
    undistortParams.fx2 = input.cameraMatrixR.at<double>(0, 0);
    undistortParams.fy2 = input.cameraMatrixR.at<double>(1, 1);
    undistortParams.cx2 = input.cameraMatrixR.at<double>(0, 2);
    undistortParams.cy2 = input.cameraMatrixR.at<double>(1, 2);
    {
        const cv::Mat& D = input.distCoeffsL;
        int n = std::min(D.rows * D.cols, 8);
        const double* dp = D.ptr<double>();
        if (n >= 1) undistortParams.k1_1 = dp[0];
        if (n >= 2) undistortParams.k2_1 = dp[1];
        if (n >= 3) undistortParams.p1_1 = dp[2];
        if (n >= 4) undistortParams.p2_1 = dp[3];
        if (n >= 5) undistortParams.k3_1 = dp[4];
    }
    {
        const cv::Mat& D = input.distCoeffsR;
        int n = std::min(D.rows * D.cols, 8);
        const double* dp = D.ptr<double>();
        if (n >= 1) undistortParams.k1_2 = dp[0];
        if (n >= 2) undistortParams.k2_2 = dp[1];
        if (n >= 3) undistortParams.p1_2 = dp[2];
        if (n >= 4) undistortParams.p2_2 = dp[3];
        if (n >= 5) undistortParams.k3_2 = dp[4];
    }
    undistortParams.imageWidth = input.leftMarkerGray.cols;
    undistortParams.imageHeight = input.leftMarkerGray.rows;
    for (int i = 0; i < 9 && i < input.R.rows * input.R.cols; ++i)
        undistortParams.R[i] = input.R.at<double>(i / 3, i % 3);
    for (int i = 0; i < 3 && i < input.T.rows; ++i)
        undistortParams.T[i] = input.T.at<double>(i);

    undistortOp.SetParams(undistortParams);
    if (!input.R1.empty()) undistortOp.SetRectifyMatrices(input.R1, input.R2, input.P1, input.P2, input.Q);

    auto undistortRes = undistortOp.Execute(mergeL.mergedEdgePoints, mergeR.mergedEdgePoints,
                                             mergeL.groupIds, mergeR.groupIds);
    if (!undistortRes.success) {
        result.message = "undistort failed";
        return result;
    }

    // ===== 2-7 ellipse_fit (per group, L/R) =====
    report(65, "ellipse_fit...");
    calib::EllipseFitCPU ellipseOp;

    auto fitEllipses = [&](const std::vector<cv::Point2d>& rectifiedPoints,
                           const std::vector<int>& groupIds, int groupCount) {
        std::vector<calib::EllipseFitCPUResult> results;
        for (int g = 0; g < groupCount; ++g) {
            std::vector<cv::Point2d> groupPoints;
            for (size_t i = 0; i < rectifiedPoints.size(); ++i)
                if (i < groupIds.size() && groupIds[i] == g)
                    groupPoints.push_back(rectifiedPoints[i]);
            if (groupPoints.size() >= 5) {
                auto er = ellipseOp.Execute(groupPoints);
                if (er.success) results.push_back(std::move(er));
            }
        }
        return results;
    };

    auto ellipsesL = fitEllipses(undistortRes.rectifiedPoints1, undistortRes.groupIds1, undistortRes.groupCount1);
    auto ellipsesR = fitEllipses(undistortRes.rectifiedPoints2, undistortRes.groupIds2, undistortRes.groupCount2);

    if (ellipsesL.empty() || ellipsesR.empty()) {
        result.message = "ellipse_fit failed or empty";
        return result;
    }

    // 提取中心（供阶段3和frame_fuse使用）
    auto extractCenters = [](const std::vector<calib::EllipseFitCPUResult>& ellipses) {
        std::vector<cv::Point2f> centers;
        for (const auto& e : ellipses) centers.emplace_back((float)e.centerX, (float)e.centerY);
        return centers;
    };
    result.leftCenters = extractCenters(ellipsesL);
    result.rightCenters = extractCenters(ellipsesR);

    // ===== 2-8 marker_match =====
    report(70, "marker_match...");
    calib::MarkerMatchCPU matchOp;
    auto matchRes = matchOp.Execute(result.leftCenters, result.rightCenters);
    if (!matchRes.success || matchRes.centerMatches.empty()) {
        result.message = "marker_match failed";
        return result;
    }

    // ===== 2-9 epipolar_intersect (L/R) =====
    report(75, "epipolar_intersect...");
    calib::EpipolarIntersectCPU epipolarOp;
    auto epiL = epipolarOp.Execute(ellipsesL);
    auto epiR = epipolarOp.Execute(ellipsesR);

    // ===== 2-10 edge_match =====
    report(80, "edge_match...");
    calib::EdgeMatchCPU edgeMatchOp;
    auto edgeMatchRes = edgeMatchOp.Execute(epiL.ellipseResults, epiR.ellipseResults, matchRes.centerMatches);
    if (!edgeMatchRes.success) {
        result.message = "edge_match failed";
        return result;
    }

    // ===== 2-11 point_reconstruct =====
    report(85, "point_reconstruct...");
    calib::PointReconstructCPU reconstructOp;
    calib::PointReconstructCPUParams reconParams;
    reconParams.fxLeft = input.cameraMatrixL.at<double>(0, 0);
    reconParams.fyLeft = input.cameraMatrixL.at<double>(1, 1);
    reconParams.cxLeft = input.cameraMatrixL.at<double>(0, 2);
    reconParams.cyLeft = input.cameraMatrixL.at<double>(1, 2);
    reconParams.fxRight = input.cameraMatrixR.at<double>(0, 0);
    reconParams.fyRight = input.cameraMatrixR.at<double>(1, 1);
    reconParams.cxRight = input.cameraMatrixR.at<double>(0, 2);
    reconParams.cyRight = input.cameraMatrixR.at<double>(1, 2);
    for (int i = 0; i < 9 && i < input.R.rows * input.R.cols; ++i)
        reconParams.R(i / 3, i % 3) = input.R.at<double>(i / 3, i % 3);
    for (int i = 0; i < 3 && i < input.T.rows; ++i)
        reconParams.T[i] = input.T.at<double>(i);
    reconstructOp.SetParams(reconParams);
    if (!input.P1.empty()) reconstructOp.SetProjectionMatrices(input.P1, input.P2, input.Q);

    auto reconRes = reconstructOp.Execute(edgeMatchRes);
    if (!reconRes.success || reconRes.markerResults.empty()) {
        result.message = "point_reconstruct failed";
        return result;
    }

    // 提取标记点3D位置和法线（供 frame_fuse 使用）
    for (const auto& mr : reconRes.markerResults) {
        if (mr.validPlane) {
            result.markerPositions.emplace_back(mr.centerX, mr.centerY, mr.centerZ);
            result.markerNormals.emplace_back(mr.normalX, mr.normalY, mr.normalZ);
        }
    }
    if (result.markerPositions.empty()) {
        result.message = "no valid 3D markers";
        return result;
    }

    // ===== 2-12 frame_fuse =====
    report(90, "frame_fuse...");
    calib::FrameFuseCPU fuseOp;

    calib::MarkerPointSet currentSet;
    currentSet.positions = result.markerPositions;
    currentSet.normals = result.markerNormals;

    if (prevFrame && prevFrame->success && !prevFrame->markerPositions.empty()) {
        calib::MarkerPointSet prevSet;
        prevSet.positions = prevFrame->markerPositions;
        prevSet.normals = prevFrame->markerNormals;
        auto fuseRes = fuseOp.Execute(currentSet, prevSet);
        if (fuseRes.success) {
            result.R = fuseRes.R;
            result.T = fuseRes.T;
            result.transform = fuseRes.transform;
        } else {
            result.message = "frame_fuse failed";
            return result;
        }
    } else {
        // 第一帧：无配准，transform=identity
        spdlog::info("pose_estimation: first frame, skip fuse");
    }

    // ===== 2-13 pose_estimate =====
    report(95, "pose_estimate...");
    calib::PoseEstimateCPU poseOp;
    auto poseRes = poseOp.Execute(result.R, result.T);
    result.anyMatched = poseRes.anyMatched;
    result.bestMatch = poseRes.bestMatch;
    if (poseRes.anyMatched && poseRes.bestMatch >= 0) {
        result.matchedPoseName = poseRes.matches[poseRes.bestMatch].targetName;
    }

    report(100, "pose_estimation done");
    result.success = true;
    return result;
}

// ============================================================================
// 激光标定（依赖相机标定结果）
// ============================================================================
LaserCalibResult runLaserCalibration(const LaserCalibInput& input,
    std::function<void(int, const std::string&)> progress)
{
    LaserCalibResult result;
    auto report = [&](int pct, const std::string& step) {
        if (progress) progress(pct, step);
    };

    // 前置检查
    if (!input.cameraCalib || !input.cameraCalib->success) {
        result.message = "camera calibration required";
        return result;
    }
    if (input.poses.empty()) {
        result.message = "no laser pose images";
        return result;
    }
    const auto& cal = *input.cameraCalib;

    report(5, "laser calibration: initializing operators...");

    // 提取相机标定参数
    double f = cal.cameraMatrixL.at<double>(0, 0);
    cv::Point2d principalPoint(cal.cameraMatrixL.at<double>(0, 2),
                                cal.cameraMatrixL.at<double>(1, 2));

    // 创建 CUDA 算子（标定参数通过 SetParams 或构造函数传入）
    calib::MaskExtractCUDA maskOp;
    calib::RegionAnalyzerCUDA cclOp;
    calib::LaserLabelerCUDA labelOp;
    calib::StegerExtractorCUDA stegerOp;
    calib::LaserMatchCuda matchOp;
    calib::LaserReconstructCuda reconstructOp;

    calib::UndistortPointsCuda undistortLeft;
    calib::UndistortPointsParams undistLeftParams;
    undistLeftParams.cameraMatrix = cal.cameraMatrixL;
    undistLeftParams.distCoeffs = cal.distCoeffsL;
    undistLeftParams.R = cal.R1;
    undistLeftParams.P = cal.P1;
    undistortLeft.SetParams(undistLeftParams);

    calib::UndistortPointsCuda undistortRight;
    calib::UndistortPointsParams undistRightParams;
    undistRightParams.cameraMatrix = cal.cameraMatrixR;
    undistRightParams.distCoeffs = cal.distCoeffsR;
    undistRightParams.R = cal.R2;
    undistRightParams.P = cal.P2;
    undistortRight.SetParams(undistRightParams);

    calib::EpipolarInterpCuda epipolarOp;
    calib::EpipolarInterpParams epipolarParams;
    epipolarParams.lineIdCheck = true;  // 标定模式
    epipolarOp.SetParams(epipolarParams);

    // 聚合容器
    std::vector<calib::PosePointSet> allPoses;
    int totalPoints = 0;

    cv::cuda::Stream stream;

    // ===== 逐姿态处理激光链 =====
    int poseIdx = 0;
    for (const auto& pose : input.poses) {
        int pct = 10 + static_cast<int>(70.0 * poseIdx / input.poses.size());
        report(pct, "pose " + std::to_string(poseIdx + 1) + "/" +
                     std::to_string(input.poses.size()));

        // --- 4-1 mask_extract (L/R) ---
        auto maskL = maskOp.Execute(pose.leftLaserGray, stream);
        auto maskR = maskOp.Execute(pose.rightLaserGray, stream);
        if (!maskL.success || !maskR.success) {
            spdlog::warn("pose {}: mask_extract failed, skip", poseIdx);
            ++poseIdx;
            continue;
        }

        // --- 4-2 CCL (L/R) ---
        auto cclL = cclOp.Execute(*maskL.d_cleanedMask, stream);
        auto cclR = cclOp.Execute(*maskR.d_cleanedMask, stream);
        if (!cclL.success || !cclR.success) {
            spdlog::warn("pose {}: CCL failed, skip", poseIdx);
            ++poseIdx;
            continue;
        }

        // --- 4-3 laser_label (L/R) ---
        auto labelL = labelOp.Execute(*cclL.d_labeledMask, stream);
        auto labelR = labelOp.Execute(*cclR.d_labeledMask, stream);
        if (!labelL.success || !labelR.success) {
            spdlog::warn("pose {}: laser_label failed, skip", poseIdx);
            ++poseIdx;
            continue;
        }

        // --- 4-4 steger (L/R, ByLabel) ---
        auto stegerL = stegerOp.Execute(*maskL.d_grayImage, *labelL.d_labeledMask, stream);
        auto stegerR = stegerOp.Execute(*maskR.d_grayImage, *labelR.d_labeledMask, stream);
        if (!stegerL.success || !stegerR.success) {
            spdlog::warn("pose {}: steger failed, skip", poseIdx);
            ++poseIdx;
            continue;
        }

        // --- 4-5 undistort (L/R) ---
        auto undistL = undistortLeft.Execute(*stegerL.d_centerPoints, *stegerL.d_line_ids, stream);
        auto undistR = undistortRight.Execute(*stegerR.d_centerPoints, *stegerR.d_line_ids, stream);
        if (!undistL.success || !undistR.success) {
            spdlog::warn("pose {}: undistort failed, skip", poseIdx);
            ++poseIdx;
            continue;
        }

        // --- 4-6 epipolar_interp (L/R, lineIdCheck=true) ---
        auto epiL = epipolarOp.Execute(*undistL.d_rectifiedPoints, *undistL.d_line_ids, stream);
        auto epiR = epipolarOp.Execute(*undistR.d_rectifiedPoints, *undistR.d_line_ids, stream);
        if (!epiL.success || !epiR.success) {
            spdlog::warn("pose {}: epipolar_interp failed, skip", poseIdx);
            ++poseIdx;
            continue;
        }

        // --- 4-7 laser_match (L/R) ---
        auto matchRes = matchOp.Execute(*epiL.d_interpPoints, *epiL.d_interp_line_ids,
                                         *epiR.d_interpPoints, *epiR.d_interp_line_ids, stream);
        if (!matchRes.success || matchRes.matchCount == 0) {
            spdlog::warn("pose {}: laser_match failed/empty, skip", poseIdx);
            ++poseIdx;
            continue;
        }

        // --- 4-8 laser_reconstruct ---
        auto recon = reconstructOp.Execute(*matchRes.d_matched_left, *matchRes.d_matched_right,
                                            *matchRes.d_matched_line_ids, cal.Q, stream);
        if (!recon.success || recon.validCount == 0) {
            spdlog::warn("pose {}: laser_reconstruct failed/empty, skip", poseIdx);
            ++poseIdx;
            continue;
        }

        // --- 聚合：download d_points3d → host ---
        stream.waitForCompletion();

        calib::PosePointSet pps;
        cv::Mat points3dHost;
        recon.d_points3d->download(points3dHost, stream);
        stream.waitForCompletion();

        // points3d is CV_32FC3, each row is a Point3f
        for (int i = 0; i < points3dHost.rows; ++i) {
            cv::Vec3f pt = points3dHost.at<cv::Vec3f>(i, 0);
            if (std::isfinite(pt[0]) && std::isfinite(pt[1]) && std::isfinite(pt[2]) &&
                std::fabs(pt[0]) < 1e5f && std::fabs(pt[1]) < 1e5f && std::fabs(pt[2]) < 1e5f) {
                pps.points3d.push_back(pt);
            }
        }

        // line_ids
        cv::Mat lineIdsHost;
        recon.d_valid_line_ids->download(lineIdsHost, stream);
        stream.waitForCompletion();
        for (int i = 0; i < lineIdsHost.rows && i < (int)pps.points3d.size(); ++i) {
            pps.lineIds.push_back(lineIdsHost.at<int>(i, 0));
        }

        if (!pps.points3d.empty()) {
            allPoses.push_back(std::move(pps));
            totalPoints += static_cast<int>(allPoses.back().points3d.size());
            spdlog::info("pose {}: {} valid 3D points", poseIdx, allPoses.back().points3d.size());
        }

        ++poseIdx;
    }

    // 清理 GPU 算子
    maskOp.Destroy();
    cclOp.Destroy();
    labelOp.Destroy();
    stegerOp.Destroy();
    matchOp.Destroy();
    reconstructOp.Destroy();
    undistortLeft.Destroy();
    undistortRight.Destroy();
    epipolarOp.Destroy();

    report(82, "aggregation done: " + std::to_string(allPoses.size()) + " poses, " +
               std::to_string(totalPoints) + " points");

    if (allPoses.size() < 3) {
        result.message = "too few valid poses: " + std::to_string(allPoses.size());
        return result;
    }

    // ===== projector_joint_calib =====
    report(85, "projector_joint_calib...");

    calib::ProjectorJointCalibParams pjcParams;
    calib::ProjectorJointCalib pjc(pjcParams);

    calib::ProjectorJointCalibInput pjcInput;
    pjcInput.poses = std::move(allPoses);
    pjcInput.f = f;
    pjcInput.principalPoint = principalPoint;
    pjcInput.initialT = cv::Vec3d(input.initialTx, input.initialTy, input.initialTz);

    auto pjcResult = pjc.Execute(pjcInput);

    if (!pjcResult.success) {
        result.message = "projector_joint_calib failed: " + pjcResult.message;
        return result;
    }

    result.projectorT[0] = pjcResult.projectorT[0];
    result.projectorT[1] = pjcResult.projectorT[1];
    result.projectorT[2] = pjcResult.projectorT[2];
    result.improvementRatio = pjcResult.improvementRatio;
    result.finalRms = pjcResult.finalSampsonRms;
    result.poseCount = pjcResult.poseCount;
    result.totalPointCount = pjcResult.totalPointCount;

    report(90, "projector_joint_calib done: T=(" +
               std::to_string(result.projectorT[0]) + "," +
               std::to_string(result.projectorT[1]) + "," +
               std::to_string(result.projectorT[2]) + ")");

    // ===== plane_map + 温度补偿表（可选，后续补充）=====
    // TODO: plane_map + plane_map_temp_table + laser_extrinsic_compensate

    report(100, "laser calibration done");
    result.success = true;
    return result;
}

// ============================================================================
// 保存/加载（JSON）
// ============================================================================
static nlohmann::json matToJson(const cv::Mat& m)
{
    nlohmann::json j;
    j["rows"] = m.rows;
    j["cols"] = m.cols;
    j["type"] = m.type();
    std::vector<double> data;
    for (int r = 0; r < m.rows; ++r)
        for (int c = 0; c < m.cols; ++c)
            data.push_back(m.at<double>(r, c));
    j["data"] = data;
    return j;
}

static cv::Mat jsonToMat(const nlohmann::json& j)
{
    int rows = j.value("rows", 0);
    int cols = j.value("cols", 0);
    int type = j.value("type", CV_64F);
    cv::Mat m(rows, cols, type);
    const auto& data = j["data"];
    int idx = 0;
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c)
            m.at<double>(r, c) = data[idx++];
    return m;
}

bool saveCalibResult(const std::string& filepath, const CameraCalibResult& r)
{
    nlohmann::json j;
    j["success"] = r.success;
    j["message"] = r.message;
    j["intrinsicRMS"] = r.intrinsicRMS;
    j["stereoReprojError"] = r.stereoReprojError;
    j["epipolarErrorMean"] = r.epipolarErrorMean;
    j["validFrameCount"] = r.validFrameCount;
    j["hasTempTables"] = r.hasTempTables;
    if (r.hasTempTables && !r.tempTablesJson.empty()) {
        j["tempTables"] = nlohmann::json::parse(r.tempTablesJson);
    }

    j["cameraMatrixL"] = matToJson(r.cameraMatrixL);
    j["distCoeffsL"] = matToJson(r.distCoeffsL);
    j["cameraMatrixR"] = matToJson(r.cameraMatrixR);
    j["distCoeffsR"] = matToJson(r.distCoeffsR);
    j["R"] = matToJson(r.R);
    j["T"] = matToJson(r.T);
    j["R1"] = matToJson(r.R1);
    j["R2"] = matToJson(r.R2);
    j["P1"] = matToJson(r.P1);
    j["P2"] = matToJson(r.P2);
    j["Q"] = matToJson(r.Q);

    std::ofstream ofs(filepath);
    if (!ofs.is_open()) return false;
    ofs << j.dump(2);
    return true;
}

bool loadCalibResult(const std::string& filepath, CameraCalibResult& r)
{
    std::ifstream ifs(filepath);
    if (!ifs.is_open()) return false;
    nlohmann::json j;
    ifs >> j;

    r.success = j.value("success", false);
    r.message = j.value("message", "");
    r.intrinsicRMS = j.value("intrinsicRMS", 0.0);
    r.stereoReprojError = j.value("stereoReprojError", 0.0);
    r.epipolarErrorMean = j.value("epipolarErrorMean", 0.0);
    r.validFrameCount = j.value("validFrameCount", 0);
    r.hasTempTables = j.value("hasTempTables", false);
    if (j.contains("tempTables")) {
        r.tempTablesJson = j["tempTables"].dump();
    }

    if (j.contains("cameraMatrixL")) r.cameraMatrixL = jsonToMat(j["cameraMatrixL"]);
    if (j.contains("distCoeffsL")) r.distCoeffsL = jsonToMat(j["distCoeffsL"]);
    if (j.contains("cameraMatrixR")) r.cameraMatrixR = jsonToMat(j["cameraMatrixR"]);
    if (j.contains("distCoeffsR")) r.distCoeffsR = jsonToMat(j["distCoeffsR"]);
    if (j.contains("R")) r.R = jsonToMat(j["R"]);
    if (j.contains("T")) r.T = jsonToMat(j["T"]);
    if (j.contains("R1")) r.R1 = jsonToMat(j["R1"]);
    if (j.contains("R2")) r.R2 = jsonToMat(j["R2"]);
    if (j.contains("P1")) r.P1 = jsonToMat(j["P1"]);
    if (j.contains("P2")) r.P2 = jsonToMat(j["P2"]);
    if (j.contains("Q")) r.Q = jsonToMat(j["Q"]);

    return r.success;
}

bool saveLaserCalibResult(const std::string& filepath, const LaserCalibResult& r)
{
    nlohmann::json j;
    j["success"] = r.success;
    j["message"] = r.message;
    j["projectorT"] = {r.projectorT[0], r.projectorT[1], r.projectorT[2]};
    j["improvementRatio"] = r.improvementRatio;
    j["finalRms"] = r.finalRms;
    j["poseCount"] = r.poseCount;
    j["totalPointCount"] = r.totalPointCount;
    j["hasTempTables"] = r.hasTempTables;
    if (r.hasTempTables && !r.tempTablesJson.empty()) {
        j["tempTables"] = nlohmann::json::parse(r.tempTablesJson);
    }
    std::ofstream ofs(filepath);
    if (!ofs.is_open()) return false;
    ofs << j.dump(2);
    return true;
}

bool loadLaserCalibResult(const std::string& filepath, LaserCalibResult& r)
{
    std::ifstream ifs(filepath);
    if (!ifs.is_open()) return false;
    nlohmann::json j;
    ifs >> j;
    r.success = j.value("success", false);
    r.message = j.value("message", "");
    if (j.contains("projectorT") && j["projectorT"].is_array()) {
        auto& t = j["projectorT"];
        r.projectorT[0] = t.size() > 0 ? t[0].get<double>() : 0;
        r.projectorT[1] = t.size() > 1 ? t[1].get<double>() : 0;
        r.projectorT[2] = t.size() > 2 ? t[2].get<double>() : 0;
    }
    r.improvementRatio = j.value("improvementRatio", 0.0);
    r.finalRms = j.value("finalRms", 0.0);
    r.poseCount = j.value("poseCount", 0);
    r.totalPointCount = j.value("totalPointCount", 0);
    r.hasTempTables = j.value("hasTempTables", false);
    if (j.contains("tempTables")) {
        r.tempTablesJson = j["tempTables"].dump();
    }
    return r.success;
}

// ============================================================================
// 完整标定流程（阶段0→2→3→4 编排）
// ============================================================================
FullCalibResult runFullCalibration(
    const CalibSessionConfig& config,
    std::function<bool(CalibFrameInput&)> getNextFrame,
    std::function<void(const CalibSessionState&)> onProgress)
{
    FullCalibResult result;
    CalibSessionState& state = result.session;
    state.targetPoseCount = static_cast<int>(config.poseTargets.size());
    if (state.targetPoseCount == 0) state.targetPoseCount = 25;
    state.poseCollected.resize(state.targetPoseCount, false);

    auto report = [&](const std::string& step) {
        state.currentStep = step;
        if (onProgress) onProgress(state);
    };

    // ===== 阶段2：姿态采集循环 =====
    report("pose collection started");

    PoseEstimationInput poseInput;
    poseInput.cameraMatrixL = config.cameraMatrixL;
    poseInput.distCoeffsL = config.distCoeffsL;
    poseInput.cameraMatrixR = config.cameraMatrixR;
    poseInput.distCoeffsR = config.distCoeffsR;
    poseInput.R = config.R;
    poseInput.T = config.T;
    poseInput.R1 = config.R1;
    poseInput.R2 = config.R2;
    poseInput.P1 = config.P1;
    poseInput.P2 = config.P2;
    poseInput.Q = config.Q;
    poseInput.chessboardCols = config.chessboardCols;
    poseInput.chessboardRows = config.chessboardRows;
    poseInput.squareSizeMm = config.squareSizeMm;

    // 采集的图像和椭圆中心（供阶段3/4使用）
    struct CollectedPose {
        std::string name;
        cv::Mat leftMarkerGray, rightMarkerGray;
        std::vector<cv::Mat> leftLaserGrays, rightLaserGrays;
        double temperature = 25.0;
        std::vector<cv::Point2f> leftCenters, rightCenters;
    };
    std::vector<CollectedPose> collectedPoses;
    PoseEstimationResult prevPoseResult;
    std::string lastHitPose;
    bool havePrev = false;

    CalibFrameInput frame;
    while (getNextFrame(frame)) {
        state.frameCount++;

        // 跳过空帧
        if (frame.leftMarkerGray.empty() || frame.rightMarkerGray.empty()) {
            report("frame " + std::to_string(state.frameCount) + ": empty, skip");
            continue;
        }

        // 单帧姿态估计
        poseInput.leftMarkerGray = frame.leftMarkerGray;
        poseInput.rightMarkerGray = frame.rightMarkerGray;

        PoseEstimationResult poseResult = runPoseEstimation(
            poseInput,
            havePrev ? &prevPoseResult : nullptr);

        prevPoseResult = poseResult;
        havePrev = true;

        if (!poseResult.success || !poseResult.anyMatched) {
            report("frame " + std::to_string(state.frameCount) + ": no pose match");
            continue;
        }

        const std::string& hitPose = poseResult.matchedPoseName;
        int poseIdx = poseResult.bestMatch;
        if (poseIdx < 0 || poseIdx >= state.targetPoseCount) {
            continue;
        }

        // 多帧确认：连续两帧命中同一姿态
        if (hitPose == lastHitPose) {
            // 确认命中，检查是否已采集
            if (!state.poseCollected[poseIdx]) {
                // 保存
                CollectedPose cp;
                cp.name = hitPose;
                cp.temperature = frame.temperature;
                cp.leftCenters = poseResult.leftCenters;
                cp.rightCenters = poseResult.rightCenters;
                // 如果有激光帧也保存
                if (!frame.leftLaserGray.empty())
                    cp.leftLaserGrays.push_back(frame.leftLaserGray.clone());
                if (!frame.rightLaserGray.empty())
                    cp.rightLaserGrays.push_back(frame.rightLaserGray.clone());

                collectedPoses.push_back(std::move(cp));
                state.poseCollected[poseIdx] = true;
                state.collectedPoses++;

                report("pose '" + hitPose + "' collected (" +
                       std::to_string(state.collectedPoses) + "/" +
                       std::to_string(state.targetPoseCount) + ")");

                spdlog::info("calib: pose '{}' collected ({}/{})",
                             hitPose, state.collectedPoses, state.targetPoseCount);

                // 检查是否集齐
                if (state.collectedPoses >= state.targetPoseCount) {
                    report("all poses collected, starting calibration...");
                    break;
                }
            }
        }

        lastHitPose = hitPose;
        report("frame " + std::to_string(state.frameCount) + ": pose '" + hitPose + "' hit (confirming...)");
    }

    if (state.collectedPoses < state.targetPoseCount) {
        result.message = "pose collection incomplete: " +
                         std::to_string(state.collectedPoses) + "/" +
                         std::to_string(state.targetPoseCount);
        report(result.message);
        return result;
    }

    // ===== 阶段3：相机标定 =====
    report("stage 3: camera calibration...");

    CameraCalibInput camInput;
    camInput.imageWidth = config.imageWidth;
    camInput.imageHeight = config.imageHeight;
    camInput.cte = config.cte;
    camInput.referenceTemp = config.referenceTemp;
    camInput.tempStep = config.tempStep;
    camInput.tempRangeMin = config.tempRangeMin;
    camInput.tempRangeMax = config.tempRangeMax;
    camInput.rectifyAlpha = -1.0;

    // 收集所有姿态的角点（使用椭圆中心作为角点）
    for (const auto& cp : collectedPoses) {
        if (!cp.leftCenters.empty())
            camInput.leftCorners.push_back(cp.leftCenters);
        if (!cp.rightCenters.empty())
            camInput.rightCorners.push_back(cp.rightCenters);
    }

    auto camReport = [&](int pct, const std::string& step) {
        report("camera: " + step);
    };
    result.cameraCalib = runCameraCalibration(camInput, camReport);
    if (!result.cameraCalib.success) {
        result.message = "camera calibration failed: " + result.cameraCalib.message;
        return result;
    }
    report("camera calibration done");

    // ===== 阶段4：激光标定 =====
    report("stage 4: laser calibration...");

    LaserCalibInput laserInput;
    laserInput.cameraCalib = &result.cameraCalib;
    laserInput.initialTx = config.initialTx;
    laserInput.initialTy = config.initialTy;
    laserInput.initialTz = config.initialTz;
    laserInput.cte = config.cte;
    laserInput.referenceTemp = config.referenceTemp;
    laserInput.tempStep = config.tempStep;
    laserInput.tempRangeMin = config.tempRangeMin;
    laserInput.tempRangeMax = config.tempRangeMax;

    for (const auto& cp : collectedPoses) {
        if (!cp.leftLaserGrays.empty() && !cp.rightLaserGrays.empty()) {
            LaserPoseImages lpi;
            lpi.leftLaserGray = cp.leftLaserGrays[0];
            lpi.rightLaserGray = cp.rightLaserGrays[0];
            lpi.temperature = cp.temperature;
            laserInput.poses.push_back(std::move(lpi));
        }
    }

    if (laserInput.poses.empty()) {
        report("laser: no laser images collected, skipping laser calibration");
        result.success = true;
        result.message = "camera calibration done, laser skipped (no laser images)";
        report("done");
        return result;
    }

    auto laserReport = [&](int pct, const std::string& step) {
        report("laser: " + step);
    };
    result.laserCalib = runLaserCalibration(laserInput, laserReport);
    if (!result.laserCalib.success) {
        result.message = "laser calibration failed: " + result.laserCalib.message;
        // 相机标定成功，激光失败不致命
        report("laser calibration failed, camera calibration OK");
    } else {
        report("laser calibration done");
    }

    result.success = result.cameraCalib.success;
    result.message = "calibration complete";
    report("done");
    return result;
}

} // namespace calibration
