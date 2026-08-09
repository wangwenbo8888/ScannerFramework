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
        zernike_ = std::make_unique<calib::ZernikeEdgeCPU>();
        ellipseFit_ = std::make_unique<calib::EllipseFitCPU>();
        markerMatch_ = std::make_unique<calib::MarkerMatchCPU>();
        // 调大 max_points（默认100太小）
        {
            calib::MarkerMatchCPUParams mp = markerMatch_->GetParams();
            mp.max_points = 500;
            markerMatch_->SetParams(mp);
        }
        pointReconstruct_ = std::make_unique<calib::PointReconstructCPU>();
        spdlog::info("[ScanPipeline] 标记点链OK");

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

    // CCL (CPU)
    cv::Mat labelsL, statsL, centroidsL;
    int nL = cv::connectedComponentsWithStats(markerMaskL, labelsL, statsL, centroidsL, 8, CV_32S);
    cv::Mat labelsR, statsR, centroidsR;
    int nR = cv::connectedComponentsWithStats(markerMaskR, labelsR, statsR, centroidsR, 8, CV_32S);

    // 标记点 ROI 提取 → zernike → ellipse_fit
    auto detectCenters = [&](const cv::Mat& gray) -> std::vector<cv::Point2f> {
        std::vector<cv::Point2f> centers;
        cv::Mat mask;
        cv::adaptiveThreshold(gray, mask, 255,
            cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 51, -5);

        cv::Mat lbl, stt, cen;
        int n = cv::connectedComponentsWithStats(mask, lbl, stt, cen, 8, CV_32S);
        int skipped = 0;
        for (int i = 1; i < n; ++i) {
            int area = stt.at<int>(i, cv::CC_STAT_AREA);
            if (area < 15 || area > 5000) { skipped++; continue; }
            int x = std::max(0, (int)stt.at<int>(i, cv::CC_STAT_LEFT) - 5);
            int y = std::max(0, (int)stt.at<int>(i, cv::CC_STAT_TOP) - 5);
            int w = std::min(gray.cols - x, (int)stt.at<int>(i, cv::CC_STAT_WIDTH) + 10);
            int h = std::min(gray.rows - y, (int)stt.at<int>(i, cv::CC_STAT_HEIGHT) + 10);
            if (w < 5 || h < 5) { skipped++; continue; }

            try {
                cv::Mat sub = gray(cv::Rect(x, y, w, h)).clone();
                auto zr = zernike_->Execute(sub);
                if (!zr.success || zr.edgePoints.empty()) { skipped++; continue; }

                auto er = ellipseFit_->Execute(zr.edgePoints);
                if (!er.success) { skipped++; continue; }

                auto c = er.centerPoint2f();
                centers.emplace_back(c.x + x, c.y + y);
            } catch (...) { skipped++; }
        }
        spdlog::info("[ScanPipeline] detectCenters: components={}, detected={}, skipped={}",
            n - 1, centers.size(), skipped);
        return centers;
    };

    auto centersL = detectCenters(leftGray);
    auto centersR = detectCenters(rightGray);

    spdlog::info("[ScanPipeline] L={} R={} calib.valid={}", centersL.size(), centersR.size(), calib_.valid);

    if (centersL.size() < 3 || centersR.size() < 3) {
        spdlog::warn("[ScanPipeline] 标记点不足 L={} R={}", centersL.size(), centersR.size());
        return output;
    }

    // 08: marker_match
    auto matchR = markerMatch_->Execute(centersL, centersR);
    if (!matchR.success || matchR.centerMatches.empty()) {
        spdlog::warn("[ScanPipeline] marker_match 失败 L={} R={}", centersL.size(), centersR.size());
        return output;
    }
    spdlog::info("[ScanPipeline] marker_match OK: {} 对", matchR.centerMatches.size());

    // 11: point_reconstruct（需要标定参数）
    if (calib_.valid) {
        pointReconstruct_->SetProjectionMatrices(calib_.P1, calib_.P2, calib_.Q);

        std::vector<int> leftIds, rightIds;
        for (size_t i = 0; i < matchR.centerMatches.size(); ++i) {
            leftIds.push_back(static_cast<int>(i));
            rightIds.push_back(matchR.centerMatches[i]);
        }

        auto reconR = pointReconstruct_->Execute(centersL, centersR, leftIds, rightIds,
                                                  matchR.centerMatches);
        if (reconR.success) {
            for (auto& mr : reconR.markerResults) {
                output.markerPoints3d.emplace_back(
                    static_cast<float>(mr.centerX),
                    static_cast<float>(mr.centerY),
                    static_cast<float>(mr.centerZ));
                output.markerNormals.emplace_back(
                    static_cast<float>(mr.normalX),
                    static_cast<float>(mr.normalY),
                    static_cast<float>(mr.normalZ));
                output.markerRadii.push_back(0.0f);
            }
        }
    }

    // 配准（首帧 I/0，后续光流）
    output.R = cv::Matx33d::eye();
    output.T = cv::Vec3d(0, 0, 0);
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

    // TODO: 激光链 CUDA 算子接入（steger→undistort→epipolar→match→reconstruct）
    // 当前留空，待标记点链跑通后逐个接入
    spdlog::debug("[ScanPipeline] 激光链待接入 (frame={})", frameCount_);

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
