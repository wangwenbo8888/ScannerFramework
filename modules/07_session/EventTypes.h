#pragma once

// ============================================================================
// EventTypes.h — 事件类型定义（事件总线用）
// 归属: modules/07_session
// ============================================================================

#include <cstdint>
#include <string>

namespace Scanner {

enum class EventType : uint16_t {
    // 设备
    DeviceConnected = 0x0100,
    DeviceDisconnected = 0x0101,
    DeviceError = 0x0102,

    // 扫描
    ScanStarted = 0x0200,
    ScanStopped = 0x0201,
    ScanPaused = 0x0202,
    ScanFrameReady = 0x0203,

    // 安全
    EmergencyStop = 0x0300,
    TemperatureUpdate = 0x0301,

    // 故障
    FaultOccurred = 0x0400,
    FaultCleared = 0x0401,

    // 会话
    SessionStarted = 0x0500,
    SessionStopped = 0x0501,
    SessionSaved = 0x0502,

    // 状态
    StateChanged = 0x0600,

    // 用户自定义起点
    UserDefined = 0x1000
};

struct Event {
    EventType type = EventType::UserDefined;
    uint32_t sourceId = 0;
    uint64_t timestamp = 0;
    int64_t param1 = 0;
    int64_t param2 = 0;
};

} // namespace Scanner
