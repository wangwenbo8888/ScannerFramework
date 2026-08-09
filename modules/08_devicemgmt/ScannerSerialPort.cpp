// ============================================================================
// ScannerSerialPort.cpp — 扫描仪下位机串口控制实现
// 仿 factory_calib/gui_qt/scanner/ScannerControl.cpp
// ============================================================================

#include "ScannerSerialPort.h"

#include <QSerialPort>
#include <QSerialPortInfo>
#include <spdlog/spdlog.h>

namespace Scanner::device {

ScannerSerialPort::ScannerSerialPort() {
    port_ = new QSerialPort();
}

ScannerSerialPort::~ScannerSerialPort() {
    close();
    delete port_;
}

// ============================================================================
// 串口枚举/打开/关闭
// ============================================================================

QStringList ScannerSerialPort::availablePorts() {
    QStringList names;
    for (const auto& info : QSerialPortInfo::availablePorts()) {
        names << info.portName();
    }
    return names;
}

bool ScannerSerialPort::open(const QString& portName) {
    // 已开则先关再开
    if (port_->isOpen()) {
        port_->clear();
        port_->close();
    }

    port_->setPortName(portName);
    if (!port_->open(QIODevice::ReadWrite)) {
        spdlog::warn("[ScannerSerialPort] 打开串口 {} 失败", portName.toStdString());
        notify(QStringLiteral("串口 %1 打开失败").arg(portName));
        return false;
    }

    // 115200/8N1/NoParity/OneStop/NoFlowControl
    port_->setBaudRate(QSerialPort::Baud115200, QSerialPort::AllDirections);
    port_->setDataBits(QSerialPort::Data8);
    port_->setFlowControl(QSerialPort::NoFlowControl);
    port_->setParity(QSerialPort::NoParity);
    port_->setStopBits(QSerialPort::OneStop);

    spdlog::info("[ScannerSerialPort] 串口 {} 已打开 (115200/8N1)", portName.toStdString());
    notify(QStringLiteral("串口 %1 已打开").arg(portName));
    return true;
}

void ScannerSerialPort::close() {
    if (port_ && port_->isOpen()) {
        port_->clear();
        port_->close();
        spdlog::info("[ScannerSerialPort] 串口已关闭");
        notify(QStringLiteral("串口已关闭"));
    }
}

bool ScannerSerialPort::isOpen() const {
    return port_ && port_->isOpen();
}

QString ScannerSerialPort::portName() const {
    return port_ ? port_->portName() : QString();
}

// ============================================================================
// 发送
// ============================================================================

bool ScannerSerialPort::send(const QString& cmd) {
    if (!port_ || !port_->isOpen()) {
        spdlog::warn("[ScannerSerialPort] 串口未打开，无法发送: {}", cmd.toStdString());
        return false;
    }
    QByteArray data = cmd.toLocal8Bit();
    qint64 written = port_->write(data);
    if (written != data.size()) {
        spdlog::warn("[ScannerSerialPort] 写入不完整: 期望 {} 实际 {}", data.size(), written);
        return false;
    }
    bool ok = port_->waitForBytesWritten(1000);
    spdlog::info("[ScannerSerialPort] 发送 '{}' written={} ok={}", cmd.toStdString(), written, ok);
    return ok;
}

// ============================================================================
// 高级命令
// ============================================================================

bool ScannerSerialPort::startScan(int freq, int bgLight, int laserLight,
                                   int trigger, int version) {
    QString cmd = QStringLiteral("N10 H%1 B%2 T%3 V%4 L%5;")
                      .arg(freq).arg(bgLight).arg(trigger).arg(version).arg(laserLight);
    fillLightOn_ = (bgLight > 0);
    laserOn_ = (laserLight > 0);
    return send(cmd);
}

bool ScannerSerialPort::stopScan() {
    fillLightOn_ = false;
    laserOn_ = false;
    return send("N11 H0;");
}

bool ScannerSerialPort::sendSoftwareTrigger() {
    return send("N12 T0;");
}

bool ScannerSerialPort::setFillLight(int level) {
    fillLightOn_ = (level > 0);
    return send(QStringLiteral("N14 B%1;").arg(level));
}

bool ScannerSerialPort::setLaser(int level) {
    laserOn_ = (level > 0);
    return send(QStringLiteral("N13 L%1;").arg(level));
}

bool ScannerSerialPort::emergencyStop() {
    return send("N15 E1;");
}

void ScannerSerialPort::setNotifyCallback(NotifyCallback cb) {
    notify_ = std::move(cb);
}

void ScannerSerialPort::notify(const QString& msg) {
    if (notify_) notify_(msg);
}

} // namespace Scanner::device
