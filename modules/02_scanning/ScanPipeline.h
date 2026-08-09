#pragma once

// ============================================================================
// ScanPipeline.h — 用户端扫描流水线（完整版，按设计文档实现）
//
// 流程:
//   灰度图(双目)
//     → 预处理 mask_separation(CUDA) → ccl(CUDA)
//       ├─ 标记点分支(CPU): split→zernike→merge→undistort→ellipse→match
//       │                  →epipolar_intersect→edge_match→reconstruct
//       └─ 激光分支(CUDA): steger→undistort→epipolar_interp→match_scan→reconstruct
//     → 配准 optical_flow_fuse (R/T)
//     → 融合 marker_cloud_fuse + laser_cloud_fuse
//     → 输出 R/T + 标记点3D + 激光3D
// ============================================================================

#include "ScanConfig.h"
#include "data/IFrameSink.h"
#include "common/types.h"
#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <memory>
#include <atomic>
#include <mutex>
#include <functional>

// 前向声明算子
namespace calib {
    class LaserMarkingSeparationCUDA;
    class RegionAnalyzerCUDA;
    class ImageSplitCPU;
    class ZernikeEdgeCPU;
    class ImageMergeCPU;
    class MarkerUndistortCPU;
    class EllipseFitCPU;
    class MarkerMatchCPU;
    class EpipolarIntersectCPU;
    class EdgeMatchCPU;
    class PointReconstructCPU;
    class MarkerOpticalFlowFuseCPU;
    class FrameFuseCPU;
    class MarkerCloudFuseCPU;
    class StegerExtractorCUDA;
    class UndistortPointsCuda;
    class EpipolarInterpCuda;
    class LaserMatchScanCuda;
    class LaserReconstructCuda;
    class LaserCloudFuseCuda;
    class LaserCloudNormalCuda;
    class LaserCloudFuseCPU;
}

namespace Scanner::workflow {

// ============================================================================
// 单帧扫描结果
// ============================================================================
struct ScanFrameOutput {
    // 标记点3D（相机坐标系）
    std::vector<cv::Point3f> markerPoints3d;
    std::vector<cv::Point3f> markerNormals;
    std::vector<float> markerRadii;

    // 激光3D点云（相机坐标系）
    std::vector<cv::Point3f> laserPoints3d;

    // 位姿（当前帧 → 全局）
    cv::Matx33d R = cv::Matx33d::eye();
    cv::Vec3d T{0, 0, 0};
    int globalMarkerCount = 0;
    bool registrationOK = false;
    bool success = false;
};

// ============================================================================
// 扫描进度回调
// ============================================================================
struct ScanProgress {
    int frameCount = 0;
    int markerCount = 0;
    int laserPointCount = 0;
    int fusedPointCount = 0;
    float fps = 0.0f;
    std::string status;
};

using ScanProgressCallback = std::function<void(const ScanProgress&)>;

// ============================================================================
// ScanPipeline — 完整扫描流水线
// ============================================================================
class ScanPipeline {
public:
    explicit ScanPipeline(const ScanConfig& config = ScanConfig::ModeB());
    ~ScanPipeline();

    ScanPipeline(const ScanPipeline&) = delete;
    ScanPipeline& operator=(const ScanPipeline&) = delete;

    // 设置标定参数
    void setCalibration(const ScanCalibration& calib);

    // 设置进度回调
    void setProgressCallback(ScanProgressCallback cb);

    // 初始化（创建算子、分配显存）
    bool initialize();

    // 处理一帧（同步）
    ScanFrameOutput processFrame(const cv::Mat& leftGray, const cv::Mat& rightGray);

    // 获取全局融合结果
    const std::vector<calib::MarkerCloudPoint>& getFusedMarkers() const;
    std::vector<cv::Point3f> getFusedLaserPoints() const;

    // 重置（开始新扫描）
    void reset();

    // 收尾全局优化（扫描结束后调用）
    bool runGlobalOptimization();

    // 配置
    const ScanConfig& config() const { return config_; }
    ScanCalibration& calibration() { return calib_; }

private:
    ScanConfig config_;
    ScanCalibration calib_;
    ScanProgressCallback callback_;
    bool initialized_ = false;

    // ---- 预处理算子 ----
    std::unique_ptr<calib::LaserMarkingSeparationCUDA> maskSep_;
    std::unique_ptr<calib::RegionAnalyzerCUDA>         ccl_;

    // ---- 标记点链算子 (CPU) ----
    std::unique_ptr<calib::ImageSplitCPU>        imgSplit_;
    std::unique_ptr<calib::ZernikeEdgeCPU>       zernike_;
    std::unique_ptr<calib::ImageMergeCPU>        imgMerge_;
    std::unique_ptr<calib::MarkerUndistortCPU>   undistortCpu_;
    std::unique_ptr<calib::EllipseFitCPU>        ellipseFit_;
    std::unique_ptr<calib::MarkerMatchCPU>       markerMatch_;
    std::unique_ptr<calib::EpipolarIntersectCPU> epipolarIntersect_;
    std::unique_ptr<calib::EdgeMatchCPU>         edgeMatch_;
    std::unique_ptr<calib::PointReconstructCPU>  pointReconstruct_;

    // ---- 配准算子 ----
    std::unique_ptr<calib::MarkerOpticalFlowFuseCPU> opticalFlow_;
    std::unique_ptr<calib::FrameFuseCPU>             frameFuse_;
    bool isFirstFrame_ = true;

    // ---- 激光链算子 (CUDA) ----
    std::unique_ptr<calib::StegerExtractorCUDA>  steger_;
    std::unique_ptr<calib::UndistortPointsCuda>  undistortCuda_;
    std::unique_ptr<calib::EpipolarInterpCuda>   epipolarInterp_;
    std::unique_ptr<calib::LaserMatchScanCuda>   laserMatch_;
    std::unique_ptr<calib::LaserReconstructCuda> laserReconstruct_;

    // ---- 融合算子 ----
    std::unique_ptr<calib::MarkerCloudFuseCPU>   markerFuse_;
    std::unique_ptr<calib::LaserCloudFuseCPU>    laserFuseCpu_;
    std::unique_ptr<calib::LaserCloudFuseCuda>  laserFuseCuda_;
    std::unique_ptr<calib::LaserCloudNormalCuda> laserNormal_;

    // ---- 全局优化 (BUILD_GLOBAL_OPTIM=ON 时启用) ----
    // std::unique_ptr<calib::GlobalBundleAdjustmentCPU> globalBA_;

    // CUDA stream
    cv::cuda::Stream cudaStream_;

    // 帧计数
    int frameCount_ = 0;

    // 内部方法
    ScanFrameOutput processMarkerBranch(
        const cv::Mat& leftGray, const cv::Mat& rightGray,
        const cv::Mat& markerMaskL, const cv::Mat& markerMaskR);

    ScanFrameOutput processLaserBranch(
        const cv::Mat& leftGray, const cv::Mat& rightGray,
        const cv::Mat& laserMaskL, const cv::Mat& laserMaskR);

    void fuseResults(ScanFrameOutput& output);

    void notifyProgress(const std::string& status);
};

} // namespace Scanner::workflow
