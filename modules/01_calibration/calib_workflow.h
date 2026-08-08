#pragma once

#include <opencv2/core.hpp>
#include <vector>
#include <string>
#include <functional>

namespace calibration {

// ============================================================================
// 固定标定板参数
// ============================================================================
constexpr int CHESSBOARD_COLS = 6;       // 横着6个点
constexpr int CHESSBOARD_ROWS = 7;       // 竖着7个点
constexpr double CHESSBOARD_SQUARE_MM = 15.0;
constexpr int CALIB_FRAME_COUNT = 25;    // 25个姿态

// ============================================================================
// 相机标定
// ============================================================================

struct CameraCalibInput {
    int imageWidth = 2048;
    int imageHeight = 1536;
    // 采集阶段已检测好的角点（每帧一组，至少25帧）
    std::vector<std::vector<cv::Point2f>> leftCorners;
    std::vector<std::vector<cv::Point2f>> rightCorners;
    // 温度补偿参数（可选，cte=0 则跳过）
    double cte = 23.6e-6;           // 热膨胀系数 (6061-T6 铝)
    double referenceTemp = 25.0;     // 参考温度 °C
    double tempStep = 2.0;           // 温度步长 °C
    double tempRangeMin = -10.0;     // 参考温度下方范围 °C（相对值）
    double tempRangeMax = 10.0;      // 参考温度上方范围 °C（相对值）
    // 立体矫正额外参数（可选）
    double rectifyAlpha = -1.0;      // -1=自动
    int rectifyFlags = 0;
};

struct CameraCalibResult {
    bool success = false;
    std::string message;

    // 内参
    cv::Mat cameraMatrixL, distCoeffsL;
    cv::Mat cameraMatrixR, distCoeffsR;
    double intrinsicRMS = 0.0;

    // 外参
    cv::Mat R, T, E, F;
    double stereoReprojError = 0.0;
    double epipolarErrorMean = 0.0;

    // 立体矫正
    cv::Mat R1, R2, P1, P2, Q;
    cv::Rect validRoiL, validRoiR;

    int validFrameCount = 0;

    // 温度补偿表（阶段3扩展，hasTempTables=true 时 tempTablesJson 有效）
    bool hasTempTables = false;
    std::string tempTablesJson;  // 序列化的3张温度补偿表（JSON字符串，供运行时解析）
};

// 完整相机标定流程（内参→外参→矫正）
// 输入：采集阶段已检测的角点
// 输出：CameraCalibResult（内存）
CameraCalibResult runCameraCalibration(const CameraCalibInput& input,
    std::function<void(int, const std::string&)> progress = nullptr);

// 保存到硬盘 / 从硬盘加载
bool saveCalibResult(const std::string& filepath, const CameraCalibResult& result);
bool loadCalibResult(const std::string& filepath, CameraCalibResult& result);

// ============================================================================
// 阶段2 姿态判断（单帧标记点处理链）
// ============================================================================

struct PoseEstimationInput {
    cv::Mat leftMarkerGray;    // 左目标记点灰度图 CV_8UC1
    cv::Mat rightMarkerGray;   // 右目标记点灰度图 CV_8UC1

    // 初始标定参数（从文件或出厂标定加载）
    cv::Mat cameraMatrixL, distCoeffsL;
    cv::Mat cameraMatrixR, distCoeffsR;
    cv::Mat R, T;              // 初始外参
    cv::Mat R1, R2, P1, P2, Q; // 初始立体矫正参数

    // 标定板参数
    int chessboardCols = 6;
    int chessboardRows = 7;
    double squareSizeMm = 15.0;
};

struct PoseEstimationResult {
    bool success = false;
    std::string message;

    // frame_fuse 输出
    cv::Matx33d R = cv::Matx33d::eye();
    cv::Vec3d T = cv::Vec3d(0, 0, 0);
    cv::Matx44d transform = cv::Matx44d::eye();

    // pose_estimate 输出
    bool anyMatched = false;
    int bestMatch = -1;
    std::string matchedPoseName;

    // 椭圆中心（供阶段3 inverse_distort 使用）
    std::vector<cv::Point2f> leftCenters;
    std::vector<cv::Point2f> rightCenters;

    // 标记点3D重建结果（供 frame_fuse 全局集使用）
    std::vector<cv::Point3d> markerPositions;
    std::vector<cv::Vec3d> markerNormals;
};

// 单帧姿态估计（14算子链：mask→filter→ccl→split→zernike→merge→undistort→ellipse→match→epipolar→edge_match→reconstruct→fuse→pose）
PoseEstimationResult runPoseEstimation(
    const PoseEstimationInput& input,
    const PoseEstimationResult* prevFrame = nullptr,
    std::function<void(int, const std::string&)> progress = nullptr);

// ============================================================================
// 激光标定（依赖相机标定结果）
// ============================================================================

struct LaserPoseImages {
    cv::Mat leftLaserGray;    // 左目激光灰度图 (CV_8UC1)
    cv::Mat rightLaserGray;   // 右目激光灰度图 (CV_8UC1)
    double temperature = 25.0;
};

struct LaserCalibInput {
    std::vector<LaserPoseImages> poses;  // 25+ 姿态的激光图像
    const CameraCalibResult* cameraCalib = nullptr;

    // 投影机光心初始值（机械装配公差，约(80,3,3)mm）
    double initialTx = 80.0;
    double initialTy = 3.0;
    double initialTz = 3.0;

    // 温度补偿（可选，cte=0 跳过）
    double cte = 0.0;
    double referenceTemp = 25.0;
    double tempStep = 2.0;
    double tempRangeMin = -10.0;
    double tempRangeMax = 10.0;
};

struct LaserCalibResult {
    bool success = false;
    std::string message;

    // projector_joint_calib 输出
    double projectorT[3] = {};
    double improvementRatio = 0.0;
    double finalRms = 0.0;
    int poseCount = 0;
    int totalPointCount = 0;

    // 温度补偿表
    bool hasTempTables = false;
    std::string tempTablesJson;
};

// 激光标定流程
LaserCalibResult runLaserCalibration(const LaserCalibInput& input,
    std::function<void(int, const std::string&)> progress = nullptr);

bool saveLaserCalibResult(const std::string& filepath, const LaserCalibResult& result);
bool loadLaserCalibResult(const std::string& filepath, LaserCalibResult& result);

} // namespace calibration
