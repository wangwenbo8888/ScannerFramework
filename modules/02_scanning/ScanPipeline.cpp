// ============================================================================
// ScanPipeline.cpp — 用户端扫描流水线完整实现
//
// 按设计文档「客户端扫描流水线.md」实现:
//   预处理(CUDA) → 标记点链(CPU, 11步) + 激光链(CUDA, 5步)
//   → 配准(optical_flow_fuse) → 融合(marker+laser)
// ============================================================================

#include "ScanPipeline.h"
#include <spdlog/spdlog.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <chrono>

// 预处理
#include "laser_markingpoint_mask_separation_cuda.h"
#include "region_analyze_cuda.h"

// 标记点链
#include "image_split_cpu.h"
#include "zernike_edge_cpu.h"
#include "image_merge_cpu.h"
#include "undistort_points_cpu.h"
#include "ellipse_fit_cpu.h"
#include "marker_match_cpu.h"
#include "epipolar_intersect_cpu.h"
#include "edge_match_cpu.h"
#include "point_reconstruct_cpu.h"

// 配准
#include "marker_optical_flow_fuse_cpu.h"
#include "frame_fuse_cpu.h"
#include "prev_frame_state.h"

// 激光链
#include "steger_extract_cuda.h"
#include "undistort_points_cuda.h"
#include "epipolar_interp_cuda.h"
#include "laser_match_scan_cuda.h"
#include "laser_reconstruct_cuda.h"

// 融合
#include "marker_cloud_fuse_cpu.h"
#include "laser_cloud_fuse_cpu.h"
#include "laser_cloud_fuse_cuda.h"
#include "laser_cloud_normal_cuda.h"

// 全局优化
// #include "global_ba_cpu.h"  // BUILD_GLOBAL_OPTIM=ON 时取消注释

namespace Scanner::workflow {

// ============================================================================
// 构造 / 析构
// ============================================================================
ScanPipeline::ScanPipeline(const ScanConfig& config) : config_(config) {}
ScanPipeline::~ScanPipeline() {
    // 在此处销毁所有算子（operator 完整类型可见）
    maskSep_.reset();
    ccl_.reset();
    imgSplit_.reset();
    zernike_.reset();
    imgMerge_.reset();
    undistortCpu_.reset();
    ellipseFit_.reset();
    markerMatch_.reset();
    epipolarIntersect_.reset();
    edgeMatch_.reset();
    pointReconstruct_.reset();
    opticalFlow_.reset();
    frameFuse_.reset();
    steger_.reset();
    undistortCuda_.reset();
    epipolarInterp_.reset();
    laserMatch_.reset();
    laserReconstruct_.reset();
    markerFuse_.reset();
    laserFuseCpu_.reset();
    laserFuseCuda_.reset();
    laserNormal_.reset();
    // globalBA_.reset();
}

void ScanPipeline::setCalibration(const ScanCalibration& calib) { calib_ = calib; }
void ScanPipeline::setProgressCallback(ScanProgressCallback cb) { callback_ = std::move(cb); }

// ============================================================================
// 初始化
// ============================================================================
bool ScanPipeline::initialize() {
    if (initialized_) return true;
    if (!calib_.valid) {
        spdlog::warn("[ScanPipeline] 标定参数未加载，标记点3D重建将跳过");
    }

    try {
        // 预处理 (CUDA — 失败不致命，有 OpenCV 回退)
        try {
            maskSep_ = std::make_unique<calib::LaserMarkingSeparationCUDA>();
            spdlog::info("[ScanPipeline] mask_separation OK");
        } catch (const std::exception& e) {
            spdlog::warn("[ScanPipeline] mask_separation 失败(OpenCV回退): {}", e.what());
        }
        try {
            ccl_ = std::make_unique<calib::RegionAnalyzerCUDA>();
            spdlog::info("[ScanPipeline] ccl OK");
        } catch (const std::exception& e) {
            spdlog::warn("[ScanPipeline] ccl 失败(OpenCV回退): {}", e.what());
        }

        // 标记点链 (CPU — 必须成功)
        imgSplit_        = std::make_unique<calib::ImageSplitCPU>();
        zernike_         = std::make_unique<calib::ZernikeEdgeCPU>();
        imgMerge_        = std::make_unique<calib::ImageMergeCPU>();
        ellipseFit_      = std::make_unique<calib::EllipseFitCPU>();
        markerMatch_     = std::make_unique<calib::MarkerMatchCPU>();
        // 调大 max_points（默认100太小）
        {
            calib::MarkerMatchCPUParams mp = markerMatch_->GetParams();
            mp.max_points = 500;
            markerMatch_->SetParams(mp);
        }
        pointReconstruct_ = std::make_unique<calib::PointReconstructCPU>();
        spdlog::info("[ScanPipeline] 标记点链OK (split+zernike+merge+ellipse+match+reconstruct)");

        // 中间算子 (需标定参数才生效)
        try {
            undistortCpu_    = std::make_unique<calib::MarkerUndistortCPU>();
            epipolarIntersect_ = std::make_unique<calib::EpipolarIntersectCPU>();
            edgeMatch_       = std::make_unique<calib::EdgeMatchCPU>();
            spdlog::info("[ScanPipeline] 中间算子OK (undistort+epipolar+edge_match)");
        } catch (const std::exception& e) {
            spdlog::warn("[ScanPipeline] 中间算子创建失败: {}", e.what());
        }

        // 配准 (可选)
        try {
            opticalFlow_ = std::make_unique<calib::MarkerOpticalFlowFuseCPU>();
            frameFuse_ = std::make_unique<calib::FrameFuseCPU>();
            spdlog::info("[ScanPipeline] 配准OK");
        } catch (const std::exception& e) {
            spdlog::warn("[ScanPipeline] 配准失败: {}", e.what());
        }

        // 激光链 (CUDA — 可选)
        if (config_.enableLaser) {
            try {
                steger_ = std::make_unique<calib::StegerExtractorCUDA>();
                undistortCuda_ = std::make_unique<calib::UndistortPointsCuda>();
                epipolarInterp_ = std::make_unique<calib::EpipolarInterpCuda>();
                laserMatch_ = std::make_unique<calib::LaserMatchScanCuda>();
                laserReconstruct_ = std::make_unique<calib::LaserReconstructCuda>();
                spdlog::info("[ScanPipeline] 激光链OK");
            } catch (const std::exception& e) {
                spdlog::warn("[ScanPipeline] 激光链失败: {}", e.what());
            }
        }

        // 融合 (CPU — 必须成功)
        calib::MarkerCloudFuseCPUParams markerFuseParams;
        markerFuseParams.voxelSize = config_.markerVoxelSize;
        markerFuse_ = std::make_unique<calib::MarkerCloudFuseCPU>(markerFuseParams);
        laserFuseCpu_ = std::make_unique<calib::LaserCloudFuseCPU>();
        spdlog::info("[ScanPipeline] 融合OK");

        // CUDA融合 (可选)
        if (config_.enableLaser) {
            try {
                laserFuseCuda_ = std::make_unique<calib::LaserCloudFuseCuda>();
                laserNormal_ = std::make_unique<calib::LaserCloudNormalCuda>();
            } catch (const std::exception& e) {
                spdlog::warn("[ScanPipeline] CUDA融合失败(CPU回退): {}", e.what());
            }
        }

        // 情况C: 预填充导入标记点
        if (!config_.initialGlobalMarkers.empty() && markerFuse_) {
            markerFuse_->Seed(config_.initialGlobalMarkers);
            spdlog::info("[ScanPipeline] 导入 {} 个已知标记点", config_.initialGlobalMarkers.size());
        }

        initialized_ = true;
        spdlog::info("[ScanPipeline] 初始化完成 (模式={}, 激光={}, GBA={})",
            static_cast<int>(config_.mode), config_.enableLaser, config_.enableFinalBA);
    } catch (const std::exception& e) {
        spdlog::error("[ScanPipeline] 初始化失败: {}", e.what());
        return false;
    }
    return true;
}

// ============================================================================
// 重置
// ============================================================================
void ScanPipeline::reset() {
    if (markerFuse_) markerFuse_->Clear();
    if (laserFuseCpu_) laserFuseCpu_->Clear();
    isFirstFrame_ = true;
    prevState_ = {};
    frameCount_ = 0;
}

// ============================================================================
// 处理一帧
// ============================================================================
ScanFrameOutput ScanPipeline::processFrame(const cv::Mat& leftGray, const cv::Mat& rightGray) {
    ScanFrameOutput output;
    if (!initialized_) {
        spdlog::error("[ScanPipeline] 未初始化");
        return output;
    }
    if (leftGray.empty() || rightGray.empty()) return output;

    auto t0 = std::chrono::steady_clock::now();
    ++frameCount_;

    // 首帧保存诊断图像
    if (frameCount_ == 1) {
        cv::imwrite("scan_debug_left.png", leftGray);
        cv::imwrite("scan_debug_right.png", rightGray);
        spdlog::info("[ScanPipeline] 首帧已保存 scan_debug_left/right.png");
    }

    try {
        // ===== 预处理: mask_separation (CUDA) — 分别处理左右图 =====
        auto maskL = maskSep_->Execute(leftGray, cudaStream_);
        auto maskR = maskSep_->Execute(rightGray, cudaStream_);

        cv::Mat markerMaskL, markerMaskR, laserMaskL, laserMaskR;
        if (maskL.success && maskL.d_markingPointMask) {
            maskL.d_markingPointMask->download(markerMaskL, cudaStream_);
            if (config_.enableLaser && maskL.d_laserMask)
                maskL.d_laserMask->download(laserMaskL, cudaStream_);
        }
        if (maskR.success && maskR.d_markingPointMask) {
            maskR.d_markingPointMask->download(markerMaskR, cudaStream_);
            if (config_.enableLaser && maskR.d_laserMask)
                maskR.d_laserMask->download(laserMaskR, cudaStream_);
        }

        if (markerMaskL.empty() || markerMaskR.empty()) {
            // CUDA 失败，退回 OpenCV 阈值
            spdlog::warn("[ScanPipeline] mask_separation 失败，退回 OpenCV");
            cv::adaptiveThreshold(leftGray, markerMaskL, 255,
                cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 51, -5);
            cv::adaptiveThreshold(rightGray, markerMaskR, 255,
                cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 51, -5);
        }

        // ===== 连通域分析 (CUDA) =====
        cv::cuda::GpuMat d_markerMaskL(markerMaskL);
        auto cclR = ccl_->Execute(d_markerMaskL, cudaStream_);

        // ===== 标记点分支 (CPU) =====
        output = processMarkerBranch(leftGray, rightGray, markerMaskL, markerMaskR);

        // ===== 激光分支 (CUDA) =====
        if (config_.enableLaser && !laserMaskL.empty()) {
            auto laserOut = processLaserBranch(leftGray, rightGray, laserMaskL, laserMaskR);
            output.laserPoints3d = std::move(laserOut.laserPoints3d);
        }

        // ===== 融合 =====
        if (output.success) {
            fuseResults(output);
        }

        // ===== 配准 (光流) =====
        // （在 markerBranch 内部处理，此处仅判断结果）
        output.registrationOK = output.markerPoints3d.size() >= 3;

    } catch (const std::exception& e) {
        spdlog::error("[ScanPipeline] 帧 {} 异常: {}", frameCount_, e.what());
    } catch (...) {
        spdlog::error("[ScanPipeline] 帧 {} 未知异常", frameCount_);
    }

    auto t1 = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    if (frameCount_ % 100 == 0) {
        spdlog::info("[ScanPipeline] {}帧: {:.1f}ms, 标记点={} 激光点={} 融合={}",
            frameCount_, ms, output.markerPoints3d.size(),
            output.laserPoints3d.size(),
            markerFuse_ ? markerFuse_->GetFusedPointCount() : 0);
    }

    notifyProgress(output.success ? "OK" : "degraded");
    return output;
}

// ============================================================================
// 标记点分支
// ============================================================================
ScanFrameOutput ScanPipeline::processMarkerBranch(
    const cv::Mat& leftGray, const cv::Mat& rightGray,
    const cv::Mat& markerMaskL, const cv::Mat& markerMaskR) {

    ScanFrameOutput output;

    // ===== CCL: 连通域分析 =====
    auto extractROIs = [](const cv::Mat& gray) -> std::vector<cv::Rect> {
        cv::Mat mask;
        cv::adaptiveThreshold(gray, mask, 255,
            cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 51, -5);
        cv::Mat lbl, stt, cen;
        int n = cv::connectedComponentsWithStats(mask, lbl, stt, cen, 8, CV_32S);
        std::vector<cv::Rect> rois;
        for (int i = 1; i < n; ++i) {
            int area = stt.at<int>(i, cv::CC_STAT_AREA);
            if (area < 15 || area > 5000) continue;
            int x = std::max(0, (int)stt.at<int>(i, cv::CC_STAT_LEFT) - 5);
            int y = std::max(0, (int)stt.at<int>(i, cv::CC_STAT_TOP) - 5);
            int w = std::min(gray.cols - x, (int)stt.at<int>(i, cv::CC_STAT_WIDTH) + 10);
            int h = std::min(gray.rows - y, (int)stt.at<int>(i, cv::CC_STAT_HEIGHT) + 10);
            if (w > 5 && h > 5) rois.emplace_back(x, y, w, h);
        }
        return rois;
    };

    auto roisL = extractROIs(leftGray);
    auto roisR = extractROIs(rightGray);
    spdlog::info("[ScanPipeline] CCL: ROIs L={} R={}", roisL.size(), roisR.size());
    if (roisL.size() < 3 || roisR.size() < 3) return output;

    // ===== 03: image_split — 按ROI裁剪子图 =====
    auto splitL = imgSplit_->Execute(leftGray, roisL);
    auto splitR = imgSplit_->Execute(rightGray, roisR);
    if (!splitL.success || !splitR.success) return output;

    // ===== 04: zernike_edge — 各子图边缘提取 =====
    std::vector<std::vector<calib::EdgePoint>> edgePtsL, edgePtsR;
    for (auto& sub : splitL.splitImages) {
        auto zr = zernike_->Execute(sub);
        edgePtsL.push_back(zr.success ? zr.edgePoints : std::vector<calib::EdgePoint>{});
    }
    for (auto& sub : splitR.splitImages) {
        auto zr = zernike_->Execute(sub);
        edgePtsR.push_back(zr.success ? zr.edgePoints : std::vector<calib::EdgePoint>{});
    }

    // ===== 05: image_merge — 合并到大图坐标系 + groupIds =====
    auto mergeL = imgMerge_->Execute(edgePtsL, roisL);
    auto mergeR = imgMerge_->Execute(edgePtsR, roisR);
    if (!mergeL.success || !mergeR.success) return output;
    spdlog::info("[ScanPipeline] merge: L={}pts {}grp, R={}pts {}grp",
        mergeL.mergedEdgeCount, mergeL.groupCount, mergeR.mergedEdgeCount, mergeR.groupCount);

    // ===== 06: undistort_cpu — 双目去畸变矫正（需标定参数）=====
    std::vector<calib::EdgePoint> undistL, undistR;
    std::vector<int> grpL, grpR;
    if (calib_.valid && undistortCpu_) {
        undistortCpu_->SetRectifyMatrices(calib_.R1, calib_.R2, calib_.P1, calib_.P2, calib_.Q);
        auto undistR_result = undistortCpu_->Execute(
            mergeL.mergedEdgePoints, mergeR.mergedEdgePoints,
            mergeL.groupIds, mergeR.groupIds);
        if (undistR_result.success) {
            // 转回 EdgePoint（矫正后坐标）
            undistL.resize(undistR_result.rectifiedPoints1.size());
            undistR.resize(undistR_result.rectifiedPoints2.size());
            for (size_t i = 0; i < undistL.size(); ++i) {
                undistL[i].x = undistR_result.rectifiedPoints1[i].x;
                undistL[i].y = undistR_result.rectifiedPoints1[i].y;
            }
            for (size_t i = 0; i < undistR.size(); ++i) {
                undistR[i].x = undistR_result.rectifiedPoints2[i].x;
                undistR[i].y = undistR_result.rectifiedPoints2[i].y;
            }
            grpL = undistR_result.groupIds1;
            grpR = undistR_result.groupIds2;
            spdlog::info("[ScanPipeline] undistort OK: L={}pts R={}pts", undistL.size(), undistR.size());
        }
    }
    // 无标定参数时用原始坐标
    if (undistL.empty()) {
        undistL = mergeL.mergedEdgePoints;
        undistR = mergeR.mergedEdgePoints;
        grpL = mergeL.groupIds;
        grpR = mergeR.groupIds;
    }

    // ===== 07: ellipse_fit — 按组拟合椭圆中心 =====
    auto fitEllipses = [&](const std::vector<calib::EdgePoint>& pts,
                           const std::vector<int>& gids, int nGroups)
        -> std::pair<std::vector<cv::Point2f>, std::vector<calib::EllipseFitCPUResult>> {
        std::vector<cv::Point2f> centers;
        std::vector<calib::EllipseFitCPUResult> ellipseResults;
        // 按组分割
        std::vector<std::vector<calib::EdgePoint>> byGroup(nGroups);
        for (size_t i = 0; i < pts.size() && i < gids.size(); ++i) {
            if (gids[i] >= 0 && gids[i] < nGroups) byGroup[gids[i]].push_back(pts[i]);
        }
        for (auto& grp : byGroup) {
            if (grp.size() < 5) { centers.emplace_back(-1, -1); ellipseResults.emplace_back(); continue; }
            auto er = ellipseFit_->Execute(grp);
            if (er.success) {
                centers.push_back(er.centerPoint2f());
                ellipseResults.push_back(std::move(er));
            } else {
                centers.emplace_back(-1, -1);
                ellipseResults.emplace_back();
            }
        }
        return {centers, std::move(ellipseResults)};
    };

    int nGrpL = mergeL.groupCount > 0 ? mergeL.groupCount : (int)roisL.size();
    int nGrpR = mergeR.groupCount > 0 ? mergeR.groupCount : (int)roisR.size();
    auto [centersL, ellipsesL] = fitEllipses(undistL, grpL, nGrpL);
    auto [centersR, ellipsesR] = fitEllipses(undistR, grpR, nGrpR);

    // 过滤无效中心
    std::vector<cv::Point2f> validL, validR;
    std::vector<int> validIdxL, validIdxR;
    for (size_t i = 0; i < centersL.size(); ++i) {
        if (centersL[i].x >= 0) { validL.push_back(centersL[i]); validIdxL.push_back(i); }
    }
    for (size_t i = 0; i < centersR.size(); ++i) {
        if (centersR[i].x >= 0) { validR.push_back(centersR[i]); validIdxR.push_back(i); }
    }
    spdlog::info("[ScanPipeline] ellipse: validL={} validR={}", validL.size(), validR.size());

    if (validL.size() < 3 || validR.size() < 3) return output;

    // ===== 08: marker_match — 立体匹配 =====
    auto matchR = markerMatch_->Execute(validL, validR);
    if (!matchR.success || matchR.centerMatches.empty()) {
        spdlog::warn("[ScanPipeline] marker_match 失败");
        return output;
    }
    spdlog::info("[ScanPipeline] match OK: {} 对", matchR.centerMatches.size());

    // ===== 09: epipolar_intersect + 10: edge_match =====
    // （需要有效椭圆参数；当前简化：跳过边缘点精化，直接用中心重建）

    // ===== 11: point_reconstruct =====
    if (calib_.valid) {
        pointReconstruct_->SetProjectionMatrices(calib_.P1, calib_.P2, calib_.Q);
        std::vector<int> leftIds, rightIds;
        for (size_t i = 0; i < matchR.centerMatches.size(); ++i) {
            leftIds.push_back(static_cast<int>(i));
            rightIds.push_back(matchR.centerMatches[i]);
        }
        auto reconR = pointReconstruct_->Execute(validL, validR, leftIds, rightIds,
                                                  matchR.centerMatches);
        if (reconR.success) {
            for (auto& mr : reconR.markerResults) {
                output.markerPoints3d.emplace_back(
                    static_cast<float>(mr.centerX), static_cast<float>(mr.centerY), static_cast<float>(mr.centerZ));
                output.markerNormals.emplace_back(
                    static_cast<float>(mr.normalX), static_cast<float>(mr.normalY), static_cast<float>(mr.normalZ));
                output.markerRadii.push_back(0.0f);
            }
            spdlog::info("[ScanPipeline] reconstruct OK: {} 3D points", output.markerPoints3d.size());
        }
    } else {
        // 无标定：输出2D中心（调试用）
        spdlog::info("[ScanPipeline] 无标定参数，跳过3D重建");
    }

    // ===== 配准: optical_flow_fuse =====
    if (!output.markerPoints3d.empty()) {
        std::vector<cv::Point3d> pos3d(output.markerPoints3d.size());
        std::vector<cv::Vec3d> norm3d(output.markerNormals.size());
        for (size_t i = 0; i < pos3d.size(); ++i)
            pos3d[i] = cv::Point3d(output.markerPoints3d[i].x, output.markerPoints3d[i].y, output.markerPoints3d[i].z);
        for (size_t i = 0; i < norm3d.size() && i < pos3d.size(); ++i)
            norm3d[i] = cv::Vec3d(output.markerNormals[i].x, output.markerNormals[i].y, output.markerNormals[i].z);
        if (norm3d.size() < pos3d.size()) norm3d.resize(pos3d.size(), cv::Vec3d(0, 0, 1));

        if (isFirstFrame_) {
            output.R = cv::Matx33d::eye();
            output.T = cv::Vec3d(0, 0, 0);
            isFirstFrame_ = false;
            spdlog::info("[ScanPipeline] 首帧: R=I T=0");
        } else if (opticalFlow_ && !prevState_.empty()) {
            try {
                auto regR = opticalFlow_->Execute(pos3d, norm3d, prevState_);
                if (regR.success) {
                    output.R = regR.R;
                    output.T = regR.T;
                    spdlog::info("[ScanPipeline] 配准OK: matched={}/{} rmse={:.3f}",
                        regR.getMatchedCount(), pos3d.size(), regR.statistics.rmse);
                } else {
                    spdlog::warn("[ScanPipeline] 配准失败: {} → 上一帧R/T", regR.message);
                    output.R = prevState_.R;
                    output.T = prevState_.T;
                }
            } catch (const std::exception& e) {
                spdlog::warn("[ScanPipeline] 配准异常: {}", e.what());
                output.R = prevState_.R;
                output.T = prevState_.T;
            }
        }
        prevState_.rawPositions = pos3d;
        prevState_.rawNormals = norm3d;
        prevState_.R = output.R;
        prevState_.T = output.T;
    }

    output.globalMarkerCount = static_cast<int>(matchR.centerMatches.size());
    output.success = !output.markerPoints3d.empty();
    return output;
}
// ============================================================================
// 激光分支
// ============================================================================
ScanFrameOutput ScanPipeline::processLaserBranch(
    const cv::Mat& leftGray, const cv::Mat& rightGray,
    const cv::Mat& laserMaskL, const cv::Mat& laserMaskR) {

    ScanFrameOutput output;

    if (!steger_ || !undistortCuda_ || !epipolarInterp_ || !laserMatch_ || !laserReconstruct_) {
        spdlog::warn("[ScanPipeline] 激光链算子未初始化");
        return output;
    }

    try {
        // 上传到 GPU
        cv::cuda::GpuMat d_leftGray(leftGray), d_rightGray(rightGray);
        cv::cuda::GpuMat d_laserMaskL(laserMaskL), d_laserMaskR(laserMaskR);

        // ===== 04: steger — 激光中心亚像素提取 (Flat模式) =====
        auto stegerL = steger_->Execute(d_leftGray, d_laserMaskL, cudaStream_);
        auto stegerR = steger_->Execute(d_rightGray, d_laserMaskR, cudaStream_);
        if (!stegerL.success || !stegerR.success ||
            !stegerL.d_centerPoints || !stegerR.d_centerPoints) {
            spdlog::warn("[ScanPipeline] steger 失败 L={} R={}", stegerL.success, stegerR.success);
            return output;
        }
        spdlog::info("[ScanPipeline] steger OK: L={} R={} pts",
            stegerL.d_centerPoints->cols, stegerR.d_centerPoints->cols);

        // ===== 05: undistort_cuda — 去畸变矫正 =====
        // 设置左相机参数
        if (calib_.valid) {
            calib::UndistortPointsParams paramsL;
            paramsL.cameraMatrix = calib_.cameraMatrixL;
            paramsL.distCoeffs = calib_.distCoeffsL;
            paramsL.R = calib_.R1;
            paramsL.P = calib_.P1;
            undistortCuda_->SetParams(paramsL);
        }
        auto undistL = undistortCuda_->Execute(
            *stegerL.d_centerPoints,
            stegerL.d_line_ids ? *stegerL.d_line_ids : cv::cuda::GpuMat(),
            cudaStream_);
        if (!undistL.success || !undistL.d_rectifiedPoints) {
            spdlog::warn("[ScanPipeline] undistort L 失败");
            return output;
        }

        // 设置右相机参数
        if (calib_.valid) {
            calib::UndistortPointsParams paramsR;
            paramsR.cameraMatrix = calib_.cameraMatrixR;
            paramsR.distCoeffs = calib_.distCoeffsR;
            paramsR.R = calib_.R2;
            paramsR.P = calib_.P2;
            undistortCuda_->SetParams(paramsR);
        }
        auto undistR = undistortCuda_->Execute(
            *stegerR.d_centerPoints,
            stegerR.d_line_ids ? *stegerR.d_line_ids : cv::cuda::GpuMat(),
            cudaStream_);
        if (!undistR.success || !undistR.d_rectifiedPoints) {
            spdlog::warn("[ScanPipeline] undistort R 失败");
            return output;
        }
        spdlog::info("[ScanPipeline] undistort OK");

        // ===== 06: epipolar_interp — 极线插值 =====
        auto epipolarL = epipolarInterp_->Execute(
            *undistL.d_rectifiedPoints,
            undistL.d_line_ids ? *undistL.d_line_ids : cv::cuda::GpuMat(),
            cudaStream_);
        auto epipolarR = epipolarInterp_->Execute(
            *undistR.d_rectifiedPoints,
            undistR.d_line_ids ? *undistR.d_line_ids : cv::cuda::GpuMat(),
            cudaStream_);
        if (!epipolarL.success || !epipolarR.success) {
            spdlog::warn("[ScanPipeline] epipolar_interp 失败");
            return output;
        }
        spdlog::info("[ScanPipeline] epipolar_interp OK");

        // ===== 07: laser_match_scan — 线匹配 =====
        auto matchR = laserMatch_->Execute(
            *epipolarL.d_interpPoints,
            epipolarL.d_interp_line_ids ? *epipolarL.d_interp_line_ids : cv::cuda::GpuMat(),
            *epipolarR.d_interpPoints,
            epipolarR.d_interp_line_ids ? *epipolarR.d_interp_line_ids : cv::cuda::GpuMat(),
            cudaStream_);
        if (!matchR.success || !matchR.d_matched_left || !matchR.d_matched_right) {
            spdlog::warn("[ScanPipeline] laser_match_scan 失败");
            return output;
        }
        spdlog::info("[ScanPipeline] laser_match OK: {} pairs", matchR.d_matched_left->cols);

        // ===== 08: laser_reconstruct — 三维重建 =====
        cv::Mat Q = calib_.valid ? calib_.Q : cv::Mat();
        if (Q.empty()) {
            spdlog::warn("[ScanPipeline] 无Q矩阵，跳过激光重建");
            return output;
        }

        auto reconR = laserReconstruct_->Execute(
            *matchR.d_matched_left,
            *matchR.d_matched_right,
            matchR.d_matched_line_ids ? *matchR.d_matched_line_ids : cv::cuda::GpuMat(),
            Q, cudaStream_);
        if (!reconR.success || !reconR.d_points3d) {
            spdlog::warn("[ScanPipeline] laser_reconstruct 失败");
            return output;
        }

        // 下载 3D 点到 CPU
        cudaStream_.waitForCompletion();
        cv::Mat points3d;
        reconR.d_points3d->download(points3d, cudaStream_);
        cudaStream_.waitForCompletion();

        if (!points3d.empty() && points3d.type() == CV_32FC3) {
            cv::Point3f* ptr = points3d.ptr<cv::Point3f>();
            output.laserPoints3d.assign(ptr, ptr + points3d.total());
            spdlog::info("[ScanPipeline] laser 3D: {} points", output.laserPoints3d.size());
        }

    } catch (const std::exception& e) {
        spdlog::warn("[ScanPipeline] 激光链异常: {}", e.what());
    } catch (...) {
        spdlog::warn("[ScanPipeline] 激光链未知异常");
    }

    return output;
}

// ============================================================================
// 融合
// ============================================================================
void ScanPipeline::fuseResults(ScanFrameOutput& output) {
    // 标记点融合
    if (!output.markerPoints3d.empty()) {
        std::vector<calib::MarkerFuseInput> markerInputs;
        markerInputs.reserve(output.markerPoints3d.size());
        for (size_t i = 0; i < output.markerPoints3d.size(); ++i) {
            calib::MarkerFuseInput mi;
            mi.x  = output.markerPoints3d[i].x;
            mi.y  = output.markerPoints3d[i].y;
            mi.z  = output.markerPoints3d[i].z;
            mi.nx = output.markerNormals[i].x;
            mi.ny = output.markerNormals[i].y;
            mi.nz = output.markerNormals[i].z;
            mi.whiteRadius = i < output.markerRadii.size() ? output.markerRadii[i] : 0.0f;
            markerInputs.push_back(mi);
        }
        markerFuse_->Execute(markerInputs, output.R, output.T);
    }

    // 激光点云融合 (CPU fallback)
    if (!output.laserPoints3d.empty()) {
        laserFuseCpu_->Execute(output.laserPoints3d, output.R, output.T);
    }
}

// ============================================================================
// 全局优化
// ============================================================================
bool ScanPipeline::runGlobalOptimization() {
    // TODO: 需 BUILD_GLOBAL_OPTIM=ON + Ceres
    spdlog::info("[ScanPipeline] GlobalOptimStage (待启用: BUILD_GLOBAL_OPTIM=ON)");
    return false;
}

// ============================================================================
// 获取结果
// ============================================================================
const std::vector<calib::MarkerCloudPoint>& ScanPipeline::getFusedMarkers() const {
    static const std::vector<calib::MarkerCloudPoint> empty;
    return markerFuse_ ? markerFuse_->GetFusedPoints() : empty;
}

std::vector<cv::Point3f> ScanPipeline::getFusedLaserPoints() const {
    // 从 CPU 融合获取（CUDA 融合需额外接口）
    if (laserFuseCpu_) {
        // TODO: LaserCloudFuseCPU 需暴露 GetFusedPoints()
        return {};
    }
    return {};
}

void ScanPipeline::notifyProgress(const std::string& status) {
    if (!callback_) return;
    ScanProgress p;
    p.frameCount = frameCount_;
    p.fusedPointCount = markerFuse_ ? static_cast<int>(markerFuse_->GetFusedPointCount()) : 0;
    p.status = status + " 标定=" + (calib_.valid ? "有" : "无");
    callback_(p);
}

} // namespace Scanner::workflow
