#include "app/WorkflowContext.h"
#include "modules/08_devicemgmt/FrameBuffer.h"
#include "modules/06_fileio/PointCloudBuffer.h"
#include "modules/08_devicemgmt/DeviceStateCache.h"
#include "modules/07_session/StateMachine.h"
#include "modules/06_fileio/ParameterManager.h"
#include "modules/07_session/SessionService.h"
#include "modules/07_session/EventBus.h"
#include "modules/08_devicemgmt/IScannerCamera.h"
#include "modules/08_devicemgmt/IMCU.h"
#include <chrono>

namespace Scanner::workflow {

WorkflowContext::WorkflowContext() {}
WorkflowContext::~WorkflowContext() {}

void WorkflowContext::publishProgress(int currentStage, int totalStages,
                                       const std::string& stageName, float progress) {
    if (!eventBus_) return;
    Event evt;
    evt.type = EventType::ScanFrameReady;
    evt.param1 = currentStage;
    evt.param2 = totalStages;
    evt.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    eventBus_->publish(evt);
}

void WorkflowContext::publishEvent(EventType type, int64_t param1, int64_t param2) {
    if (!eventBus_) return;
    Event evt;
    evt.type = type;
    evt.param1 = param1;
    evt.param2 = param2;
    evt.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    eventBus_->publish(evt);
}

} // namespace Scanner::workflow
