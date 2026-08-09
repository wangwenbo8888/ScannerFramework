// ============================================================================
// AppContext.cpp — 应用层装配实现
// ============================================================================

#include "AppContext.h"
#include "data/FrameBuffer.h"
#include "data/PointCloudBuffer.h"
#include "data/DeviceStateCache.h"
#include "data/CalibStore.h"
#include "service/StateMachine.h"
#include "service/ParameterManager.h"
#include "service/FaultHandler.h"
#include "service/SessionService.h"
#include "infra/EventBus.h"
#include "modules/08_devicemgmt/CameraControl.h"
#include "modules/08_devicemgmt/MCUDriver.h"
#include "modules/08_devicemgmt/HardwareMonitor.h"
#include "modules/08_devicemgmt/ScannerSerialPort.h"
#include "workflow/WorkflowContext.h"
#include "ScanWorkflow.h"
#include "CalibrationWorkflow.h"
#include "PostProcessWorkflow.h"
#include <spdlog/spdlog.h>

AppContext::AppContext() {}
AppContext::~AppContext() { shutdown(); }

void AppContext::initialize() {
    // === Infra ===
    eventBus_ = std::make_unique<Scanner::infra::EventBus>();

    // === Data ===
    frameBuffer_      = std::make_unique<Scanner::data::FrameBuffer>(60);
    pointCloudBuffer_ = std::make_unique<Scanner::data::PointCloudBuffer>();
    deviceStateCache_ = std::make_unique<Scanner::data::DeviceStateCache>();
    calibStore_       = std::make_unique<Scanner::data::CalibStore>();

    // === Service ===
    stateMachine_   = std::make_unique<Scanner::service::StateMachine>(eventBus_.get());
    paramManager_   = std::make_unique<Scanner::service::ParameterManager>();
    faultHandler_   = std::make_unique<Scanner::service::FaultHandler>();
    sessionService_ = std::make_unique<Scanner::service::SessionService>();

    faultHandler_->setEventBus(eventBus_.get());
    faultHandler_->setStateMachine(stateMachine_.get());
    faultHandler_->start();

    // === HAL ===
    Scanner::device::StereoPairConfig camCfg;
    camCfg.deviceIndexLeft = 0;
    camCfg.deviceIndexRight = 1;
    camCfg.rotateRight180 = true;
    camera_ = std::make_unique<Scanner::device::CameraControl>(camCfg);
    mcu_    = std::make_unique<Scanner::device::MCUDriver>(115200);
    scannerSerial_ = std::make_unique<Scanner::device::ScannerSerialPort>();

    hwMonitor_ = std::make_unique<Scanner::device::HardwareMonitor>();
    hwMonitor_->setDeviceStateCache(deviceStateCache_.get());
    hwMonitor_->setEventBus(eventBus_.get());
    hwMonitor_->setMCU(mcu_.get());
    hwMonitor_->setCamera(camera_.get());

    // === 启动时自动检测并初始化设备 ===

    // 1) 检测串口，打开第一个可用串口（下位机 MCU）
    auto ports = Scanner::device::ScannerSerialPort::availablePorts();
    spdlog::info("[AppContext] 检测到串口: {}", ports.join(", ").toStdString());
    for (const auto& portName : ports) {
        if (scannerSerial_->open(portName)) {
            serialReady_ = true;
            spdlog::info("[AppContext] 下位机串口 {} 已连接", portName.toStdString());
            break;
        }
    }
    if (!serialReady_) {
        spdlog::warn("[AppContext] 未找到可用串口，下位机控制不可用");
    }

    // 2) 检测相机，尝试打开
    auto camResult = camera_->open();
    if (camResult.success) {
        cameraReady_ = true;
        spdlog::info("[AppContext] 相机已连接: {}", camResult.message);
    } else {
        cameraReady_ = false;
        spdlog::warn("[AppContext] 相机连接失败: {}", camResult.message);
    }

    // === WorkflowContext 装配 ===
    wfCtx_ = std::make_unique<Scanner::workflow::WorkflowContext>();
    wfCtx_->setFrameBuffer(frameBuffer_.get());
    wfCtx_->setPointCloudBuffer(pointCloudBuffer_.get());
    wfCtx_->setDeviceStateCache(deviceStateCache_.get());
    wfCtx_->setCalibStore(calibStore_.get());
    wfCtx_->setStateMachine(stateMachine_.get());
    wfCtx_->setParameterManager(paramManager_.get());
    wfCtx_->setSessionService(sessionService_.get());
    wfCtx_->setCamera(camera_.get());
    wfCtx_->setMCU(mcu_.get());
    wfCtx_->setEventBus(eventBus_.get());

    // === Workflow ===
    scanWf_  = std::make_unique<Scanner::workflow::ScanWorkflow>(wfCtx_.get());
    calibWf_ = std::make_unique<Scanner::workflow::CalibrationWorkflow>(wfCtx_.get());
    postWf_  = std::make_unique<Scanner::workflow::PostProcessWorkflow>(wfCtx_.get());

    spdlog::info("[AppContext] 全部组件装配完成");

    // 启动 HardwareMonitor（始终运行，周期采集设备状态）
    hwMonitor_->start(1000);
    spdlog::info("[AppContext] HardwareMonitor 已启动");
}

void AppContext::shutdown() {
    if (hwMonitor_) hwMonitor_->stop();
    if (scanWf_)    scanWf_->stop();
    if (calibWf_)   calibWf_->stop();
    if (postWf_)    postWf_->stop();
    if (faultHandler_) faultHandler_->stop();
    if (camera_)    { camera_->stopAsyncCapture(); camera_->close(); }
    if (scannerSerial_) scannerSerial_->close();
    if (mcu_)       mcu_->close();
    spdlog::info("[AppContext] 全部组件已关闭");
}
