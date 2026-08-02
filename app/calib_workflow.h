#pragma once

#include <opencv2/core.hpp>
#include <vector>
#include <string>
#include <functional>

namespace calibration {

// ============================================================================
// 输入/输出结构（采集部分由调用方提供，这里只处理算法）
// ============================================================================

struct CameraCalibInput {
    int imageWidth = 2048;
    int imageHeight = 1536;
    int chessboardCols = 11;        // 棋盘格内角点列数
    int chessboardRows = 8;         // 棋盘格内角点行数
    double squareSizeMM = 15.0;     // 棋盘格方格边长(mm)
    std::vector<cv::Mat> leftImages;   // 左相机图像（调用方采集）
    std::vector<cv::Mat> rightImages;  // 右相机图像（调用方采集）
};

struct CameraCalibResult {
    bool success = false;
    std::string message;

    // 内参结果
    cv::Mat cameraMatrixL, distCoeffsL;
    cv::Mat cameraMatrixR, distCoeffsR;
    double intrinsicRMS = 0.0;

    // 外参结果
    cv::Mat R, T, E, F;
    double stereoReprojError = 0.0;
    double epipolarErrorMean = 0.0;

    // 立体矫正结果
    cv::Mat R1, R2, P1, P2, Q;
    cv::Rect validRoiL, validRoiR;

    int validFrameCount = 0;
};

// ============================================================================
// 标定函数（独立调用，不包含采集）
// ============================================================================

// 相机标定完整流程：角点检测 → 内参 → 外参 → 立体矫正
// 输入：左右图像 + 棋盘格参数
// 输出：内参、外参、矫正参数
CameraCalibResult runCameraCalibration(const CameraCalibInput& input,
    std::function<void(int, const std::string&)> progress = nullptr);

// 仅内参标定（独立可用）
bool runIntrinsicCalibration(
    const std::vector<cv::Mat>& leftImages,
    const std::vector<cv::Mat>& rightImages,
    int chessboardCols, int chessboardRows, double squareSizeMM,
    int imageWidth, int imageHeight,
    cv::Mat& cameraMatrixL, cv::Mat& distCoeffsL,
    cv::Mat& cameraMatrixR, cv::Mat& distCoeffsR,
    double& rmsError,
    std::function<void(int, const std::string&)> progress = nullptr);

// 仅外参标定（需要内参结果）
bool runExtrinsicCalibration(
    const std::vector<cv::Mat>& leftImages,
    const std::vector<cv::Mat>& rightImages,
    int chessboardCols, int chessboardRows, double squareSizeMM,
    const cv::Mat& cameraMatrixL, const cv::Mat& distCoeffsL,
    const cv::Mat& cameraMatrixR, const cv::Mat& distCoeffsR,
    cv::Mat& R, cv::Mat& T, cv::Mat& E, cv::Mat& F,
    double& stereoReprojError, double& epipolarErrorMean,
    std::function<void(int, const std::string&)> progress = nullptr);

// 仅立体矫正（需要内参+外参结果）
bool runStereoRectify(
    const cv::Mat& cameraMatrixL, const cv::Mat& distCoeffsL,
    const cv::Mat& cameraMatrixR, const cv::Mat& distCoeffsR,
    const cv::Mat& R, const cv::Mat& T,
    int imageWidth, int imageHeight,
    cv::Mat& R1, cv::Mat& R2, cv::Mat& P1, cv::Mat& P2, cv::Mat& Q,
    cv::Rect& validRoiL, cv::Rect& validRoiR,
    std::function<void(int, const std::string&)> progress = nullptr);

// 保存标定结果到 JSON
bool saveCalibResult(const std::string& filepath, const CameraCalibResult& result);

// 加载标定结果
bool loadCalibResult(const std::string& filepath, CameraCalibResult& result);

} // namespace calibration
