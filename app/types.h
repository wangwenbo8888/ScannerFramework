#pragma once

// ============================================================================
// types.h — 框架公共类型定义（精简版）
//
// 仅保留跨模块共享的基础类型。模块专属类型已拆出:
//   EventType/Event       → modules/07_session/EventTypes.h
//   DeviceState           → modules/08_devicemgmt/DeviceTypes.h
//   ScanMode              → modules/02_scanning/ScanConfig.h
// ============================================================================

#include <cstdint>
#include <string>
#include <memory>
#include <vector>
#include <functional>

namespace Scanner {

// 质量标记
enum class QualityFlag : uint8_t {
    Normal, Degraded, Warning, Fault
};

// 版本契约分级
enum class ContractLevel : uint8_t {
    Stable, Internal, Experimental
};

// 故障严重级别
enum class FaultSeverity : uint8_t {
    Info, Warning, Error, Critical
};

// Result — 算子/操作返回值
struct Result {
    bool success = true;
    int32_t errorCode = 0;
    std::string message;
    QualityFlag qualityFlag = QualityFlag::Normal;

    static Result ok(const std::string& msg = "") { return {true, 0, msg, QualityFlag::Normal}; }
    static Result fail(int32_t code, const std::string& msg) { return {false, code, msg, QualityFlag::Fault}; }
    static Result fail(const std::string& msg) { return {false, -1, msg, QualityFlag::Fault}; }
    static Result degraded(const std::string& msg = "") { return {true, 1, msg, QualityFlag::Degraded}; }
    static Result warning(const std::string& msg = "") { return {true, 2, msg, QualityFlag::Warning}; }

    bool isDegraded() const { return qualityFlag == QualityFlag::Degraded; }
    bool hasWarning() const { return qualityFlag == QualityFlag::Warning; }
    bool isFault() const { return qualityFlag == QualityFlag::Fault; }
};

// 帧号 / 时间戳
using FrameId = uint64_t;
using TimestampMs = uint64_t;

// 位姿
struct Pose {
    double R[9] = {1,0,0, 0,1,0, 0,0,1};
    double t[3] = {0, 0, 0};
    FrameId frameId = 0;
    TimestampMs timestamp = 0;
    void identity() { R[0]=1;R[1]=0;R[2]=0;R[3]=0;R[4]=1;R[5]=0;R[6]=0;R[7]=0;R[8]=1; t[0]=t[1]=t[2]=0; }
};

// 智能指针别名
template<typename T> using UPtr = std::unique_ptr<T>;
template<typename T> using SPtr = std::shared_ptr<T>;

} // namespace Scanner
