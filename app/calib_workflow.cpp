#include "calib_workflow.h"

#include "intrinsic_calib_cpu.h"
#include "extrinsic_calib_cpu.h"
#include "stereo_rectify_cpu.h"
#include "stereo_rectify_temp_table_cpu.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <cstdio>

namespace calibration {

// ============================================================================
// 辅助：从图像检测棋盘格角点
// ============================================================================
static bool detectCorners(
    const std::vector<cv::Mat>& images,
    cv::Size patternSize,
    std::vector<std::vector<cv::Point2f>>& allCorners)
{
    allCorners.clear();
    allCorners.reserve(images.size());

    for (size_t i = 0; i < images.size(); ++i) {
        if (images[i].empty()) continue;

        cv::Mat gray;
        if (images[i].channels() == 3)
            cv::cvtColor(images[i], gray, cv::COLOR_BGR2GRAY);
        else
            gray = images[i];

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(gray, patternSize, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

        if (found) {
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01));
            allCorners.push_back(std::move(corners));
        }
    }
    return !allCorners.empty();
}

// ============================================================================
// 辅助：生成棋盘格世界坐标
// ============================================================================
static std::vector<cv::Point3f> generateObjectPoints(
    cv::Size patternSize, double squareSize)
{
    std::vector<cv::Point3f> pts;
    for (int r = 0; r < patternSize.height; ++r)
        for (int c = 0; c < patternSize.width; ++c)
            pts.emplace_back(c * squareSize, r * squareSize, 0.0f);
    return pts;
}

// ============================================================================
// 完整相机标定流程
// ============================================================================
CameraCalibResult runCameraCalibration(
    const CameraCalibInput& input,
    std::function<void(int, const std::string&)> progress)
{
    CameraCalibResult result;
    auto report = [&](int pct, const std::string& step) {
        if (progress) progress(pct, step);
    };

    cv::Size patternSize(input.chessboardCols, input.chessboardRows);
    cv::Size imageSize(input.imageWidth, input.imageHeight);

    // ===== 1. 检测角点 =====
    report(5, "\xe8\xa7\x92\xe7\x82\xb9\xe6\xa3\x80\xe6\xb5\x8b\xe4\xb8\xad...");  // 角点检测中...

    std::vector<std::vector<cv::Point2f>> leftCorners, rightCorners;
    if (!detectCorners(input.leftImages, patternSize, leftCorners) ||
        !detectCorners(input.rightImages, patternSize, rightCorners))
    {
        result.message = "\xe6\xa3\x8b\xe7\x9b\x98\xe6\xa0\xbc\xe8\xa7\x92\xe7\x82\xb9\xe6\xa3\x80\xe6\xb5\x8b\xe5\xa4\xb1\xe8\xb4\xa5";  // 棋盘格角点检测失败
        return result;
    }

    int validFrames = std::min(leftCorners.size(), rightCorners.size());
    if (validFrames < 5) {
        result.message = "\xe6\x9c\x89\xe6\x95\x88\xe5\xb8\xa7\xe6\x95\xb0\xe4\xb8\x8d\xe8\xb6\xb3: " + std::to_string(validFrames);  // 有效帧数不足
        return result;
    }
    leftCorners.resize(validFrames);
    rightCorners.resize(validFrames);
    result.validFrameCount = validFrames;

    report(15, "\xe8\xa7\x92\xe7\x82\xb9\xe6\xa3\x80\xe6\xb5\x8b\xe5\xae\x8c\xe6\x88\x90: " + std::to_string(validFrames) + " \xe5\xb8\xa7");

    // ===== 2. 内参标定 =====
    report(20, "\xe5\x86\x85\xe5\x8f\x82\xe6\xa0\x87\xe5\xae\x9a\xe4\xb8\xad...");  // 内参标定中...

    calib::IntrinsicCalibParams intrinsicParams;
    intrinsicParams.chessboard_width = input.chessboardCols;
    intrinsicParams.chessboard_height = input.chessboardRows;
    intrinsicParams.square_size_mm = input.squareSizeMM;
    intrinsicParams.image_width = input.imageWidth;
    intrinsicParams.image_height = input.imageHeight;

    calib::IntrinsicCalibCPU intrinsicCalib(intrinsicParams);
    calib::IntrinsicCalibResult intrinsicResult;

    if (!intrinsicCalib.Execute(leftCorners, rightCorners, intrinsicResult) ||
        !intrinsicResult.success)
    {
        result.message = intrinsicResult.message;
        return result;
    }

    result.cameraMatrixL = intrinsicResult.left.camera_matrix.clone();
    result.distCoeffsL = intrinsicResult.left.dist_coeffs.clone();
    result.cameraMatrixR = intrinsicResult.right.camera_matrix.clone();
    result.distCoeffsR = intrinsicResult.right.dist_coeffs.clone();
    result.intrinsicRMS = intrinsicResult.reproj_error_mean;

    report(50, "\xe5\x86\x85\xe5\x8f\x82\xe6\xa0\x87\xe5\xae\x9a\xe5\xae\x8c\xe6\x88\x90 RMS=" + std::to_string(result.intrinsicRMS));

    // ===== 3. 外参标定 =====
    report(55, "\xe5\xa4\x96\xe5\x8f\x82\xe6\xa0\x87\xe5\xae\x9a\xe4\xb8\xad...");  // 外参标定中...

    auto objectPoints = generateObjectPoints(patternSize, input.squareSizeMM);

    calib::ExtrinsicCalibCpuParams extrinsicParams;
    extrinsicParams.leftPointsPerView = leftCorners;
    extrinsicParams.rightPointsPerView = rightCorners;
    extrinsicParams.objectPoints = objectPoints;
    extrinsicParams.imageSize = imageSize;
    extrinsicParams.patternSize = patternSize;
    extrinsicParams.squareSize = static_cast<float>(input.squareSizeMM);

    calib::ExtrinsicCalibCpu extrinsicCalib(extrinsicParams);
    auto extrinsicResult = extrinsicCalib.Execute(
        result.cameraMatrixL, result.distCoeffsL,
        result.cameraMatrixR, result.distCoeffsR);

    if (!extrinsicResult.success) {
        result.message = extrinsicResult.message;
        return result;
    }

    result.R = extrinsicResult.R.clone();
    result.T = extrinsicResult.T.clone();
    result.E = extrinsicResult.E.clone();
    result.F = extrinsicResult.F.clone();
    result.stereoReprojError = extrinsicResult.stereoReprojError;
    result.epipolarErrorMean = extrinsicResult.epipolarErrorMean;

    report(75, "\xe5\xa4\x96\xe5\x8f\x82\xe6\xa0\x87\xe5\xae\x9a\xe5\xae\x8c\xe6\x88\x90");  // 外参标定完成

    // ===== 4. 立体矫正 =====
    report(80, "\xe7\xab\x8b\xe4\xbd\x93\xe7\x9f\xab\xe6\xad\xa3\xe4\xb8\xad...");  // 立体矫正中...

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
        result.message = rectifyResult.message;
        return result;
    }

    result.R1 = rectifyResult.R1.clone();
    result.R2 = rectifyResult.R2.clone();
    result.P1 = rectifyResult.P1.clone();
    result.P2 = rectifyResult.P2.clone();
    result.Q = rectifyResult.Q.clone();
    result.validRoiL = rectifyResult.validRoiLeft;
    result.validRoiR = rectifyResult.validRoiRight;

    report(100, "\xe6\xa0\x87\xe5\xae\x9a\xe5\xae\x8c\xe6\x88\x90");  // 标定完成
    result.success = true;
    return result;
}

// ============================================================================
// 仅内参标定
// ============================================================================
bool runIntrinsicCalibration(
    const std::vector<cv::Mat>& leftImages,
    const std::vector<cv::Mat>& rightImages,
    int chessboardCols, int chessboardRows, double squareSizeMM,
    int imageWidth, int imageHeight,
    cv::Mat& cameraMatrixL, cv::Mat& distCoeffsL,
    cv::Mat& cameraMatrixR, cv::Mat& distCoeffsR,
    double& rmsError,
    std::function<void(int, const std::string&)> progress)
{
    cv::Size patternSize(chessboardCols, chessboardRows);

    std::vector<std::vector<cv::Point2f>> leftCorners, rightCorners;
    if (!detectCorners(leftImages, patternSize, leftCorners) ||
        !detectCorners(rightImages, patternSize, rightCorners))
        return false;

    int n = std::min(leftCorners.size(), rightCorners.size());
    if (n < 5) return false;
    leftCorners.resize(n);
    rightCorners.resize(n);

    calib::IntrinsicCalibParams params;
    params.chessboard_width = chessboardCols;
    params.chessboard_height = chessboardRows;
    params.square_size_mm = squareSizeMM;
    params.image_width = imageWidth;
    params.image_height = imageHeight;

    calib::IntrinsicCalibCPU calib(params);
    calib::IntrinsicCalibResult result;

    if (!calib.Execute(leftCorners, rightCorners, result) || !result.success)
        return false;

    cameraMatrixL = result.left.camera_matrix.clone();
    distCoeffsL = result.left.dist_coeffs.clone();
    cameraMatrixR = result.right.camera_matrix.clone();
    distCoeffsR = result.right.dist_coeffs.clone();
    rmsError = result.reproj_error_mean;
    return true;
}

// ============================================================================
// 仅外参标定
// ============================================================================
bool runExtrinsicCalibration(
    const std::vector<cv::Mat>& leftImages,
    const std::vector<cv::Mat>& rightImages,
    int chessboardCols, int chessboardRows, double squareSizeMM,
    const cv::Mat& cameraMatrixL, const cv::Mat& distCoeffsL,
    const cv::Mat& cameraMatrixR, const cv::Mat& distCoeffsR,
    cv::Mat& R, cv::Mat& T, cv::Mat& E, cv::Mat& F,
    double& stereoReprojError, double& epipolarErrorMean,
    std::function<void(int, const std::string&)> progress)
{
    cv::Size patternSize(chessboardCols, chessboardRows);
    cv::Size imageSize;
    if (!leftImages.empty()) imageSize = leftImages[0].size();

    std::vector<std::vector<cv::Point2f>> leftCorners, rightCorners;
    if (!detectCorners(leftImages, patternSize, leftCorners) ||
        !detectCorners(rightImages, patternSize, rightCorners))
        return false;

    int n = std::min(leftCorners.size(), rightCorners.size());
    if (n < 5) return false;
    leftCorners.resize(n);
    rightCorners.resize(n);

    auto objectPoints = generateObjectPoints(patternSize, squareSizeMM);

    calib::ExtrinsicCalibCpuParams params;
    params.leftPointsPerView = leftCorners;
    params.rightPointsPerView = rightCorners;
    params.objectPoints = objectPoints;
    params.imageSize = imageSize;
    params.patternSize = patternSize;
    params.squareSize = static_cast<float>(squareSizeMM);

    calib::ExtrinsicCalibCpu calib(params);
    auto result = calib.Execute(cameraMatrixL, distCoeffsL, cameraMatrixR, distCoeffsR);

    if (!result.success) return false;

    R = result.R.clone();
    T = result.T.clone();
    E = result.E.clone();
    F = result.F.clone();
    stereoReprojError = result.stereoReprojError;
    epipolarErrorMean = result.epipolarErrorMean;
    return true;
}

// ============================================================================
// 仅立体矫正
// ============================================================================
bool runStereoRectify(
    const cv::Mat& cameraMatrixL, const cv::Mat& distCoeffsL,
    const cv::Mat& cameraMatrixR, const cv::Mat& distCoeffsR,
    const cv::Mat& R, const cv::Mat& T,
    int imageWidth, int imageHeight,
    cv::Mat& R1, cv::Mat& R2, cv::Mat& P1, cv::Mat& P2, cv::Mat& Q,
    cv::Rect& validRoiL, cv::Rect& validRoiR,
    std::function<void(int, const std::string&)> progress)
{
    calib::StereoRectifyCpuParams params;
    params.cameraMatrixL = cameraMatrixL;
    params.distCoeffsL = distCoeffsL;
    params.cameraMatrixR = cameraMatrixR;
    params.distCoeffsR = distCoeffsR;
    params.imageSize = cv::Size(imageWidth, imageHeight);
    params.R = R;
    params.T = T;

    calib::StereoRectifyCpu rectify(params);
    auto result = rectify.Execute();

    if (!result.success) return false;

    R1 = result.R1.clone();
    R2 = result.R2.clone();
    P1 = result.P1.clone();
    P2 = result.P2.clone();
    Q = result.Q.clone();
    validRoiL = result.validRoiLeft;
    validRoiR = result.validRoiRight;
    return true;
}

// ============================================================================
// 保存/加载标定结果
// ============================================================================
bool saveCalibResult(const std::string& filepath, const CameraCalibResult& result)
{
    nlohmann::json j;
    j["success"] = result.success;
    j["message"] = result.message;
    j["intrinsicRMS"] = result.intrinsicRMS;
    j["stereoReprojError"] = result.stereoReprojError;
    j["epipolarErrorMean"] = result.epipolarErrorMean;
    j["validFrameCount"] = result.validFrameCount;

    // cv::Mat → JSON 辅助
    auto matToJson = [](const cv::Mat& m) -> nlohmann::json {
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
    };

    j["cameraMatrixL"] = matToJson(result.cameraMatrixL);
    j["distCoeffsL"] = matToJson(result.distCoeffsL);
    j["cameraMatrixR"] = matToJson(result.cameraMatrixR);
    j["distCoeffsR"] = matToJson(result.distCoeffsR);
    j["R"] = matToJson(result.R);
    j["T"] = matToJson(result.T);
    j["R1"] = matToJson(result.R1);
    j["R2"] = matToJson(result.R2);
    j["P1"] = matToJson(result.P1);
    j["P2"] = matToJson(result.P2);
    j["Q"] = matToJson(result.Q);

    std::ofstream ofs(filepath);
    if (!ofs.is_open()) return false;
    ofs << j.dump(2);
    return true;
}

bool loadCalibResult(const std::string& filepath, CameraCalibResult& result)
{
    std::ifstream ifs(filepath);
    if (!ifs.is_open()) return false;
    nlohmann::json j;
    ifs >> j;

    result.success = j.value("success", false);
    result.message = j.value("message", "");
    result.intrinsicRMS = j.value("intrinsicRMS", 0.0);
    result.stereoReprojError = j.value("stereoReprojError", 0.0);
    result.epipolarErrorMean = j.value("epipolarErrorMean", 0.0);
    result.validFrameCount = j.value("validFrameCount", 0);

    auto jsonToMat = [](const nlohmann::json& j) -> cv::Mat {
        int rows = j.value("rows", 0);
        int cols = j.value("cols", 0);
        int type = j.value("type", CV_64F);
        cv::Mat m(rows, cols, type);
        auto data = j["data"];
        int idx = 0;
        for (int r = 0; r < rows; ++r)
            for (int c = 0; c < cols; ++c)
                m.at<double>(r, c) = data[idx++];
        return m;
    };

    if (j.contains("cameraMatrixL")) result.cameraMatrixL = jsonToMat(j["cameraMatrixL"]);
    if (j.contains("distCoeffsL")) result.distCoeffsL = jsonToMat(j["distCoeffsL"]);
    if (j.contains("cameraMatrixR")) result.cameraMatrixR = jsonToMat(j["cameraMatrixR"]);
    if (j.contains("distCoeffsR")) result.distCoeffsR = jsonToMat(j["distCoeffsR"]);
    if (j.contains("R")) result.R = jsonToMat(j["R"]);
    if (j.contains("T")) result.T = jsonToMat(j["T"]);
    if (j.contains("R1")) result.R1 = jsonToMat(j["R1"]);
    if (j.contains("R2")) result.R2 = jsonToMat(j["R2"]);
    if (j.contains("P1")) result.P1 = jsonToMat(j["P1"]);
    if (j.contains("P2")) result.P2 = jsonToMat(j["P2"]);
    if (j.contains("Q")) result.Q = jsonToMat(j["Q"]);

    return result.success;
}

} // namespace calibration
