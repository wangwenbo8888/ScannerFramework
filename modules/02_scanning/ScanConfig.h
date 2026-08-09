#pragma once

// ============================================================================
// ScanConfig.h — 扫描流水线配置（三模式开关）
//
// 设计文档 §3.3: 单一 ScanPipeline + ScanConfig
// ============================================================================

#include <vector>
#include <opencv2/core.hpp>
#include "marker_cloud_fuse_cpu.h"  // MarkerFuseInput

namespace Scanner::workflow {

// ============================================================================
// 扫描模式
// ============================================================================
enum class ScanMode {
    MarkerOnly,     // 情况A: 纯标记点
    MarkerLaser,    // 情况B: 标记点+激光
    MarkerLaserImported  // 情况C: 标记点+激光+导入标记点
};

// ============================================================================
// 扫描标定参数（由标定流程产出）
// ============================================================================
struct ScanCalibration {
    cv::Mat cameraMatrixL, distCoeffsL;
    cv::Mat cameraMatrixR, distCoeffsR;
    cv::Mat R1, R2, P1, P2, Q;
    cv::Size imageSize{2048, 1536};
    bool valid = false;

    // 温度补偿表（可选）
    bool hasTempCompensation = false;
};

// ============================================================================
// ScanConfig — 三模式开关
// ============================================================================
struct ScanConfig {
    ScanMode mode = ScanMode::MarkerLaser;

    // 开关1: 激光链 (情况A=false, B/C=true)
    bool enableLaser = true;

    // 开关2: 导入已知标记点 (情况C)
    std::vector<calib::MarkerFuseInput> initialGlobalMarkers;

    // 开关3: 收尾全局优化
    bool enableFinalBA = false;  // 情况A必跑, B/C可选

    // 曝光参数
    double exposureMarkerMs = 50.0;   // 标志点帧曝光
    double exposureLaserMs  = 10.0;   // 激光帧曝光

    // 激光参数
    int numLaserLines = 4;

    // 体素融合参数
    float markerVoxelSize = 0.5f;     // mm
    float laserVoxelSize  = 0.5f;     // mm

    static ScanConfig ModeA() {
        ScanConfig c;
        c.mode = ScanMode::MarkerOnly;
        c.enableLaser = false;
        c.enableFinalBA = true;
        return c;
    }

    static ScanConfig ModeB() {
        ScanConfig c;
        c.mode = ScanMode::MarkerLaser;
        c.enableLaser = true;
        c.enableFinalBA = false;
        return c;
    }

    static ScanConfig ModeC() {
        ScanConfig c;
        c.mode = ScanMode::MarkerLaserImported;
        c.enableLaser = true;
        c.enableFinalBA = false;
        return c;
    }
};

} // namespace Scanner::workflow
