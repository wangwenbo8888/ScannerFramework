#include "common/types.h"
#include "infra/EventBus.h"
#include "infra/Scheduler.h"
#include "service/IState.h"
#include "workflow/IWorkflow.h"
#include "workflow/Pipeline.h"
#include "data/RingBuffer.h"

#include <iostream>
#include <string>
#include <thread>
#include <chrono>

using namespace Scanner;

// ============================================================================
// 简易状态机实现
// ============================================================================
class SimpleScannerState : public service::IState {
public:
    service::ScannerState getCurrentState() const override { return state_; }
    std::string getStateName() const override { return stateNames_[static_cast<int>(state_)]; }

    Result transition(EventType event, int64_t) override {
        switch (event) {
            case EventType::DeviceConnected:
                if (state_ == service::ScannerState::Init) {
                    state_ = service::ScannerState::DeviceReady;
                    return Result::ok("Device ready");
                }
                return Result::fail("Cannot connect in current state");

            case EventType::ScanStarted:
                if (state_ == service::ScannerState::Calibrated) {
                    state_ = service::ScannerState::Scanning;
                    return Result::ok("Scanning started");
                }
                return Result::fail("Must calibrate first");

            case EventType::ScanStopped:
                if (state_ == service::ScannerState::Scanning) {
                    state_ = service::ScannerState::PostProcessing;
                    return Result::ok("Post-processing");
                }
                return Result::fail("Not scanning");

            case EventType::EmergencyStop:
                state_ = service::ScannerState::EmergencyStop;
                return Result::ok("Emergency stop activated");

            default:
                return Result::ok("Event ignored");
        }
    }

    bool canOperate(const std::string& operation) const override {
        if (operation == "scan") return state_ == service::ScannerState::Calibrated;
        if (operation == "calibrate") return state_ == service::ScannerState::DeviceReady;
        return true;
    }

private:
    service::ScannerState state_ = service::ScannerState::Init;
    const char* stateNames_[9] = {
        "Init", "DeviceReady", "Calibrating", "Calibrated",
        "Scanning", "Paused", "PostProcessing", "Error", "EmergencyStop"
    };
};

// ============================================================================
// 简易工作流实现
// ============================================================================
class SimpleWorkflow : public workflow::IWorkflow {
public:
    explicit SimpleWorkflow(std::string name) : name_(std::move(name)) {}

    std::string getName() const override { return name_; }

    Result initialize() override {
        std::cout << "  [" << name_ << "] Initialized" << std::endl;
        state_ = workflow::WorkflowState::Idle;
        return Result::ok();
    }

    Result start() override {
        state_ = workflow::WorkflowState::Running;
        std::cout << "  [" << name_ << "] Started" << std::endl;
        progress_ = 0.0f;
        return Result::ok();
    }

    Result pause() override {
        if (state_ == workflow::WorkflowState::Running) {
            state_ = workflow::WorkflowState::Paused;
            return Result::ok();
        }
        return Result::fail("Not running");
    }

    Result resume() override {
        if (state_ == workflow::WorkflowState::Paused) {
            state_ = workflow::WorkflowState::Running;
            return Result::ok();
        }
        return Result::fail("Not paused");
    }

    Result stop() override {
        state_ = workflow::WorkflowState::Stopping;
        std::cout << "  [" << name_ << "] Stopped" << std::endl;
        state_ = workflow::WorkflowState::Completed;
        return Result::ok();
    }

    workflow::WorkflowState getState() const override { return state_; }

    Result setProgressCallback(workflow::WorkflowCallback cb) override {
        callback_ = std::move(cb);
        return Result::ok();
    }

    void reportProgress(const std::string& stage, float progress) {
        if (callback_) {
            workflow::WorkflowProgress wp;
            wp.state = state_;
            wp.stageName = stage;
            wp.progress = progress;
            wp.message = stage;
            callback_(wp);
        }
    }

private:
    std::string name_;
    workflow::WorkflowState state_ = workflow::WorkflowState::Idle;
    workflow::WorkflowCallback callback_;
    float progress_ = 0.0f;
};

// ============================================================================
// 主程序
// ============================================================================
int main() {
    std::cout << "======================================" << std::endl;
    std::cout << "  3D Scanner Framework v1.0" << std::endl;
    std::cout << "======================================" << std::endl;
    std::cout << std::endl;

    // 1. 初始化 EventBus
    infra::EventBus eventBus;
    std::cout << "[1] EventBus initialized (subscribers: "
              << eventBus.getSubscriberCount() << ")" << std::endl;

    // 2. 初始化 Scheduler
    infra::Scheduler scheduler(4);
    std::cout << "[2] Scheduler initialized (threads: "
              << scheduler.getThreadCount() << ")" << std::endl;

    // 3. 初始化状态机
    SimpleScannerState stateMachine;
    std::cout << "[3] State machine initialized (state: "
              << stateMachine.getStateName() << ")" << std::endl;

    // 4. 创建工作流
    SimpleWorkflow scanWorkflow("ScanWorkflow");
    scanWorkflow.initialize();
    std::cout << "[4] Workflow created: " << scanWorkflow.getName() << std::endl;

    // 5. 设置进度回调
    scanWorkflow.setProgressCallback([](const workflow::WorkflowProgress& p) {
        std::cout << "    Progress: " << p.stageName
                  << " (" << static_cast<int>(p.progress * 100) << "%)" << std::endl;
    });

    // 6. 模拟设备连接
    std::cout << std::endl << "--- Simulating device connection ---" << std::endl;
    auto     result = stateMachine.transition(EventType::DeviceConnected, 0);
    std::cout << "  Device connected: " << stateMachine.getStateName()
              << " (" << result.message << ")" << std::endl;

    // 7. 订阅事件
    auto subId = eventBus.subscribe(EventType::ScanFrameReady, [](const Event& e) {
        std::cout << "  [Event] Frame ready: frameId=" << e.param1 << std::endl;
    });
    std::cout << "  Subscribed to ScanFrameReady (id: " << subId << ")" << std::endl;

    // 8. 启动扫描工作流
    std::cout << std::endl << "--- Starting scan workflow ---" << std::endl;
    scanWorkflow.start();

    // 9. 模拟帧处理
    for (int i = 1; i <= 5; ++i) {
        Event frameEvent;
        frameEvent.type = EventType::ScanFrameReady;
        frameEvent.param1 = i;
        frameEvent.timestamp = static_cast<TimestampMs>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
        eventBus.publish(frameEvent);

        float progress = static_cast<float>(i) / 5.0f;
        scanWorkflow.reportProgress("Processing frame " + std::to_string(i), progress);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 10. 停止工作流
    scanWorkflow.stop();
    std::cout << "  Workflow state: "
              << static_cast<int>(scanWorkflow.getState()) << std::endl;

    // 11. 取消订阅
    eventBus.unsubscribe(subId);
    std::cout << "  Unsubscribed (remaining subscribers: "
              << eventBus.getSubscriberCount() << ")" << std::endl;

    // 12. 关闭 Scheduler
    scheduler.shutdown();
    std::cout << "  Scheduler shut down" << std::endl;

    std::cout << std::endl << "======================================" << std::endl;
    std::cout << "  Framework demo complete" << std::endl;
    std::cout << "======================================" << std::endl;

    return 0;
}
