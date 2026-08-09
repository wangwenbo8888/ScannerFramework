#pragma once

// ============================================================================
// ScannerSerialPort.h — 扫描仪下位机串口控制（QSerialPort）
//
// 协议格式（分号结尾）:
//   N10 H{频率} B{补光} T{触发} V{版本} L{激光};  开始扫描
//   N11 H0;                                      停止扫描
//   N12 T0;                                      软件触发
//   N13 L{亮度};                                 激光亮度
//   N14 B{亮度};                                 补光灯亮度
//   N15 E1;                                      急停
// ============================================================================

#include <QString>
#include <QStringList>
#include <functional>

class QSerialPort;
class QSerialPortInfo;

namespace Scanner::device {

class ScannerSerialPort {
public:
    ScannerSerialPort();
    ~ScannerSerialPort();

    // 串口枚举/打开/关闭
    static QStringList availablePorts();
    bool open(const QString& portName);
    void close();
    bool isOpen() const;
    QString portName() const;

    // 发送原始命令（分号结尾），返回是否成功
    bool send(const QString& cmd);

    // ---- 高级命令 ----

    // 开始扫描: N10 H{freq} B{bg} T1 V2 L{laser};
    bool startScan(int freq, int bgLight, int laserLight,
                   int trigger = 1, int version = 2);

    // 停止扫描: N11 H0;
    bool stopScan();

    // 软件触发: N12 T0;
    bool sendSoftwareTrigger();

    // 补光灯: N14 B{level};
    bool setFillLight(int level);        // level=0 关
    bool fillLightOn() const { return fillLightOn_; }

    // 激光: N13 L{level};
    bool setLaser(int level);            // level=0 关
    bool laserOn() const { return laserOn_; }

    // 急停: N15 E1;
    bool emergencyStop();

    // 通知回调（用于 UI 状态显示）
    using NotifyCallback = std::function<void(const QString&)>;
    void setNotifyCallback(NotifyCallback cb);

private:
    QSerialPort* port_ = nullptr;
    NotifyCallback notify_;
    bool fillLightOn_ = false;
    bool laserOn_ = false;

    void notify(const QString& msg);
};

} // namespace Scanner::device
