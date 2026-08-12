#pragma once

// ============================================================================
// DeviceTypes.h — 设备状态类型定义
// 归属: modules/08_devicemgmt
// ============================================================================

#include <cstdint>

namespace Scanner {

enum class DeviceState : uint8_t {
    Offline,
    Connected,
    Streaming,
    Error
};

} // namespace Scanner
