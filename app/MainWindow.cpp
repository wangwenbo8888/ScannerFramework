#include "MainWindow.h"
#include "AppContext.h"
#include "data/DeviceStateCache.h"
#include "data/PointCloudBuffer.h"
#include "CalibDialog.h"
#include "calib_workflow.h"
#include "CalibDisplay.h"
#include "IntegrateTestDialog.h"
#include "ScannerWindow.h"
#include "stubs/LEADSCANSeries.h"
#include "stubs/CameraControl.h"
#include "stubs/scan_workflow.h"
#include "MCUDriver.h"
#include "ScannerSerialPort.h"
#include "ScanPipeline.h"
#include "file_io.h"
#include <spdlog/spdlog.h>
#include <osg/Vec3>
#include <osg/Matrix>
#include <osgGA/TrackballManipulator>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <QPainter>
#include <QPainterPath>
#include <QApplication>
#include <QHeaderView>
#include <QScreen>
#include <QResizeEvent>
#include <QTimer>
#include <QThread>
#include <QScrollArea>
#include <QShortcut>
#include <QProgressDialog>
#include <QMessageBox>
#include <QMenu>
#include <QFileDialog>
#include <QStatusBar>
#include <QSerialPortInfo>
#include <QSerialPort>
#include <QTextEdit>
#include <QDateTime>
#include <cstdio>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#pragma execution_character_set("utf-8")

QPixmap MainWindow::renderSvg(const QString &svgPath, int size)
{
    return renderSvg(svgPath, size, size);
}

QPixmap MainWindow::renderSvg(const QString &svgPath, int w, int h)
{
    QSvgRenderer renderer(svgPath);
    qreal dpr = qApp->devicePixelRatio();
    QPixmap pix(QSize(int(w * dpr), int(h * dpr)));
    pix.fill(Qt::transparent);
    pix.setDevicePixelRatio(dpr);
    QPainter painter(&pix);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
    renderer.render(&painter);
    return pix;
}

ArrowSlider::ArrowSlider(Qt::Orientation orientation, QWidget *parent)
    : QSlider(orientation, parent)
{
}

void ArrowSlider::setGroovePixmap(const QPixmap &pixmap)
{
    m_groovePixmap = pixmap;
    update();
}

void ArrowSlider::paintEvent(QPaintEvent *event)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    QStyleOptionSlider opt;
    initStyleOption(&opt);
    QRect handleRect = style()->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderHandle, this);

    if (orientation() == Qt::Vertical) {
        // 竖排：渐变槽竖向，箭头指向左
        if (!m_groovePixmap.isNull()) {
            QPixmap scaled = m_groovePixmap.scaled(24, height() - 20, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
            int gx = (width() - scaled.width()) / 2;
            p.drawPixmap(gx, 10, scaled);
        }
        int cy = handleRect.center().y();
        int grooveX = (width() - 24) / 2;
        int cx = grooveX + 24;
        p.setBrush(QBrush(Qt::white));
        p.setPen(QPen(QColor(80, 80, 80), 1));
        QPolygon arrow;
        arrow << QPoint(cx, cy - 4)
              << QPoint(cx - 24, cy)
              << QPoint(cx, cy + 4);
        p.drawPolygon(arrow);
    } else {
        // 横排：渐变槽横向(旋转90度)，圆头，箭头指向下
        if (!m_groovePixmap.isNull()) {
            QTransform rotate;
            rotate.rotate(90);
            QPixmap rotated = m_groovePixmap.transformed(rotate, Qt::SmoothTransformation);
            int gw = width() - 20;
            int gh = 24;
            QPixmap scaled = rotated.scaled(gw, gh, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
            int gy = (height() - gh) / 2;
            int radius = gh / 2;
            QPainterPath clip;
            clip.addRoundedRect(QRectF(10, gy, gw, gh), radius, radius);
            p.setClipPath(clip);
            p.drawPixmap(10, gy, scaled);
            p.setClipping(false);
        }
        int cx = handleRect.center().x();
        int grooveY = (height() - 24) / 2;
        int cy = grooveY;
        p.setBrush(QBrush(Qt::white));
        p.setPen(QPen(QColor(80, 80, 80), 1));
        QPolygon arrow;
        arrow << QPoint(cx - 4, cy)
              << QPoint(cx, cy + 24)
              << QPoint(cx + 4, cy);
        p.drawPolygon(arrow);
    }
}

MainWindow::MainWindow(AppContext* appCtx, QWidget *parent) : QMainWindow(parent), m_appCtx(appCtx)
{
    setObjectName("mainWindow");
    setWindowTitle(QStringLiteral("LeadScan K2"));
    setWindowFlags(Qt::FramelessWindowHint);

    QWidget *central = new QWidget();
    central->setObjectName("centralWidget");
    QVBoxLayout *mainLayout = new QVBoxLayout(central);
    mainLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->setSpacing(0);

    mainLayout->addWidget(createTitleBar());
    mainLayout->addWidget(createNavBar());
    mainLayout->addWidget(createToolBar());

    QHBoxLayout *contentLayout = new QHBoxLayout();
    contentLayout->setContentsMargins(0, 0, 0, 0);
    contentLayout->setSpacing(0);
    contentLayout->addWidget(createLeftPanel(), 2);
    contentLayout->addWidget(create3DViewArea(), 5);
    mainLayout->addLayout(contentLayout, 1);

    setCentralWidget(central);
    
    createFloatingToolbar();

    m_integrateTestDialog = nullptr;
    m_calibDialog = nullptr;

    // 显示设备连接状态
    if (m_appCtx) {
        QStringList statusParts;
        statusParts << QStringLiteral("相机: %1").arg(m_appCtx->cameraReady() ? "已连接" : "未连接");
        statusParts << QStringLiteral("串口: %1").arg(
            m_appCtx->scannerSerial() && m_appCtx->scannerSerial()->isOpen() ?
            m_appCtx->scannerSerial()->portName() : "未连接");
        statusBar()->showMessage(statusParts.join("  |  "));
        spdlog::info("[MainWindow] 设备状态: camera={} serial={}",
            m_appCtx->cameraReady(),
            m_appCtx->scannerSerial() && m_appCtx->scannerSerial()->isOpen());
    }

    // 调试日志弹窗
    m_debugLogDlg = new QDialog(this);
    m_debugLogDlg->setWindowTitle(QStringLiteral("调试日志"));
    m_debugLogDlg->setMinimumSize(600, 300);
    m_debugLogDlg->setWindowFlags(Qt::Window);
    auto* dbgLayout = new QVBoxLayout(m_debugLogDlg);
    m_debugLogText = new QTextEdit(m_debugLogDlg);
    m_debugLogText->setReadOnly(true);
    m_debugLogText->setStyleSheet("QTextEdit { background-color: #1e1e1e; color: #4ec9b0; font-family: Consolas; font-size: 12px; }");
    dbgLayout->addWidget(m_debugLogText);
    m_debugLogDlg->show();

    startInfoTimer();
}

void MainWindow::appendDebugLog(const QString& msg) {
    if (!m_debugLogText) return;
    QString ts = QDateTime::currentDateTime().toString("hh:mm:ss.zzz");
    m_debugLogText->append(QStringLiteral("[%1] %2").arg(ts, msg));
    spdlog::info("[DebugLog] {}", msg.toStdString());
}

MainWindow::~MainWindow() {
    m_scanning = false;
    if (m_scanThread.joinable()) m_scanThread.join();
}

void MainWindow::onIntegrateTestClicked()
{
    if (!m_integrateTestDialog) {
        auto* dlg = new ScannerWindow(m_appCtx, this);
        m_integrateTestDialog = dlg;
    }
    m_integrateTestDialog->show();
    m_integrateTestDialog->setWindowState(Qt::WindowActive);
    m_integrateTestDialog->raise();
    m_integrateTestDialog->activateWindow();
}

void MainWindow::onReloadPointCloud()
{
    m_3dView->clearScene();
    if (m_cloudItem001)
        m_cloudItem001->setText(0, QStringLiteral("点云数据 001 (0)"));
    m_3dView->loadTestDataFromPLY("D:/pointcloud_100M.ply", 0);
}

void MainWindow::onCalibDeviceClicked()
{
    // ===== 一键完整标定流程 =====
    // 设置标定显示视图（扫描仪模型 + 姿态引导）
    if (m_3dView && m_3dViewArea) {
        if (m_floatingToolbar) {
            m_floatingToolbar->setVisible(false);
            m_floatingToolbar->hide();
            m_floatingToolbar->move(-10000, -10000);
        }

        std::string stlTarget = "E:/workfold/framework/build/JEAMMSCAN.stl";
        osg::ref_ptr<osg::Group> scene = calib_display::buildCalibScene(stlTarget);
        m_3dView->setSceneData(scene);
        m_3dView->setCenterOverlayVisible(false);

        auto* manip = new osgGA::TrackballManipulator();
        osg::BoundingSphere bs = scene->getBound();
        double dist = 500.0;
        manip->setHomePosition(
            osg::Vec3(bs.center().x(), bs.center().y(), bs.center().z() + dist),
            bs.center(),
            osg::Vec3(0.0, 1.0, 0.0)
        );
        m_3dView->setCameraManipulator(manip);
        manip->home(0);
        m_3dView->viewer()->frame();
        osg::Matrix lockedView = m_3dView->viewer()->getCamera()->getViewMatrix();
        m_3dView->setCameraManipulator(nullptr);
        m_3dView->viewer()->getCamera()->setViewMatrix(lockedView);
    }

    // 创建/显示 2D 标定板 + 姿态偏差彩条
    if (!m_calibBoard2D) {
        m_calibBoard2D = new calib_display::CalibBoard2D();
        auto* viewContainer = m_3dView ? m_3dView->parentWidget() : nullptr;
        auto* hLayout = viewContainer ? qobject_cast<QHBoxLayout*>(viewContainer->layout()) : nullptr;
        auto* viewArea = viewContainer ? viewContainer->parentWidget() : nullptr;
        auto* vLayout = viewArea ? qobject_cast<QVBoxLayout*>(viewArea->layout()) : nullptr;

        // 标定板加入右侧布局（和3D视图并排，各占一半）
        if (hLayout) {
            m_calibBoard2D->setMinimumWidth(300);
            m_calibBoard2D->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
            hLayout->addWidget(m_calibBoard2D, 1);
        }

        if (vLayout) {
            auto* lrWidget = new QWidget();
            lrWidget->setObjectName("calibLrBar");
            lrWidget->setFixedHeight(60);
            auto* lrLayout = new QHBoxLayout(lrWidget);
            lrLayout->setContentsMargins(0, 10, 0, 10);
            lrLayout->setSpacing(4);
            lrLayout->addStretch(1);
            auto* labelL = new QLabel(QStringLiteral("\xe5\xb7\xa6"));
            labelL->setAlignment(Qt::AlignCenter);
            labelL->setStyleSheet("color: #0066FF; font-size: 14px; font-weight: bold;");
            auto* lrSlider = new ArrowSlider(Qt::Horizontal);
            lrSlider->setRange(0, 100);
            lrSlider->setValue(50);
            lrSlider->setFixedHeight(48);
            lrSlider->setMinimumWidth(200);
            lrSlider->setGroovePixmap(renderSvg(":/icons/resources/icons/div.color-gradient-bar.svg", 800, 24));
            auto* labelR = new QLabel(QStringLiteral("\xe5\x8f\xb3"));
            labelR->setAlignment(Qt::AlignCenter);
            labelR->setStyleSheet("color: #FF0000; font-size: 14px; font-weight: bold;");
            lrLayout->addWidget(labelL);
            lrLayout->addWidget(lrSlider, 8);
            lrLayout->addWidget(labelR);
            lrLayout->addStretch(1);
            vLayout->insertWidget(0, lrWidget);

            auto* fbWidget = new QWidget();
            fbWidget->setObjectName("calibFbBar");
            fbWidget->setFixedWidth(60);
            auto* fbLayout = new QVBoxLayout(fbWidget);
            fbLayout->setContentsMargins(10, 0, 10, 0);
            fbLayout->setSpacing(4);
            auto* labelF = new QLabel(QStringLiteral("\xe5\x89\x8d"));
            labelF->setAlignment(Qt::AlignCenter);
            labelF->setStyleSheet("color: #0066FF; font-size: 14px; font-weight: bold;");
            auto* fbSlider = new ArrowSlider(Qt::Vertical);
            fbSlider->setRange(0, 100);
            fbSlider->setValue(50);
            fbSlider->setFixedWidth(48);
            fbSlider->setMinimumHeight(200);
            fbSlider->setGroovePixmap(renderSvg(":/icons/resources/icons/div.color-gradient-bar.svg", 24, 800));
            auto* labelB = new QLabel(QStringLiteral("\xe5\x90\x8e"));
            labelB->setAlignment(Qt::AlignCenter);
            labelB->setStyleSheet("color: #FF0000; font-size: 14px; font-weight: bold;");
            fbLayout->addWidget(labelF);
            fbLayout->addWidget(fbSlider, 1);
            fbLayout->addWidget(labelB);
            if (hLayout) hLayout->addWidget(fbWidget);
        }
    }

    if (m_calibBoard2D) m_calibBoard2D->show();
    {
        auto* va = m_3dView ? m_3dView->parentWidget()->parentWidget() : nullptr;
        if (va) { auto* b = va->findChild<QWidget*>("calibLrBar"); if (b) b->show(); }
        auto* vc = m_3dView ? m_3dView->parentWidget() : nullptr;
        if (vc) { auto* b = vc->findChild<QWidget*>("calibFbBar"); if (b) b->show(); }
    }

    auto* cam = m_appCtx ? m_appCtx->camera() : nullptr;
    auto* scannerSerial = m_appCtx ? m_appCtx->scannerSerial() : nullptr;

    // 诊断：显示设备连接状态
    QString diag;
    diag += QStringLiteral("相机: %1\n").arg(cam && cam->isOpen() ? "已连接" : "未连接");
    diag += QStringLiteral("串口: %1\n").arg(
        scannerSerial && scannerSerial->isOpen() ? scannerSerial->portName() : "未连接");
    spdlog::info("calib diag: camera_open={} serial_open={}",
        cam && cam->isOpen(), scannerSerial && scannerSerial->isOpen());
    statusBar()->showMessage(diag.replace("\n", " | "));
    QApplication::processEvents();

    if (!cam || !cam->isOpen()) {
        QMessageBox::warning(this, QStringLiteral("标定"), QStringLiteral("相机未连接，请检查设备"));
        return;
    }

    // 确保异步采集正在运行（连续模式）
    if (!cam->isCapturing()) {
        statusBar()->showMessage(QStringLiteral("正在启动相机采集..."));
        QApplication::processEvents();
        cam->setExposure(5.0);
        auto r = cam->startAsyncCaptureContinuous([](const Scanner::hal::StereoFrame&) {});
        spdlog::info("calib: startAsyncCaptureContinuous success={} msg={}", r.success, r.message);
        if (!r.success) {
            QMessageBox::warning(this, QStringLiteral("标定"),
                QStringLiteral("相机采集启动失败: %1").arg(QString::fromStdString(r.message)));
            return;
        }
    }

    // 测试抓一帧
    statusBar()->showMessage(QStringLiteral("测试相机采集..."));
    QApplication::processEvents();
    QThread::msleep(500); // 等回调产帧
    {
        Scanner::hal::StereoFrame testFrame;
        auto r = cam->grabFrame(testFrame, 5000);
        spdlog::info("calib: test grab success={} leftEmpty={} rightEmpty={} msg={}",
                     r.success, testFrame.leftGray.empty(), testFrame.rightGray.empty(), r.message);
        if (!r.success || testFrame.leftGray.empty()) {
            QMessageBox::warning(this, QStringLiteral("标定"),
                QStringLiteral("相机采集测试失败: %1\n请检查相机连接").arg(QString::fromStdString(r.message)));
            return;
        }
        // 保存测试帧
        cv::imwrite("calib_test_left.png", testFrame.leftGray);
        cv::imwrite("calib_test_right.png", testFrame.rightGray);
        spdlog::info("calib: test frames saved, size={}x{}", testFrame.leftGray.cols, testFrame.leftGray.rows);
    }

    // ===== 进度对话框 =====
    QProgressDialog progress(QStringLiteral("准备标定..."), QStringLiteral("取消"),
                             0, 25, this);
    progress.setWindowTitle(QStringLiteral("一键标定"));
    progress.setWindowModality(Qt::NonModal);
    progress.setMinimumDuration(0);
    progress.setMinimumWidth(500);
    progress.move(100, 100);
    progress.show();
    progress.activateWindow();
    progress.raise();

    // ===== 相机预览弹窗（放在屏幕右侧，不挡进度对话框）=====
    calib_display::CameraPreviewDialog previewDlg(this);
    previewDlg.setWindowTitle(QStringLiteral("相机预览"));
    previewDlg.resize(700, 350);
    previewDlg.move(650, 100);
    previewDlg.show();

    auto updatePreview = [&](const cv::Mat& left, const cv::Mat& right, const QString& status) {
        if (!left.empty()) previewDlg.updateFrames(left, right, status);
        progress.setLabelText(status);
        statusBar()->showMessage(status);
        QApplication::processEvents();
    };

    auto showCalibError = [&](const QString& msg) {
        spdlog::error("calib error: {}", msg.toStdString());
        statusBar()->showMessage(QStringLiteral("错误: %1").arg(msg));
        previewDlg.lower();
        QMessageBox::critical(&previewDlg, QStringLiteral("标定错误"), msg);
    };

    // 使用已初始化的串口发送命令
    auto sendMcu = [&](const QString& cmd) -> bool {
        if (!scannerSerial || !scannerSerial->isOpen()) return false;
        return scannerSerial->send(cmd);
    };

    // 开补光灯（N10 启动命令：补光60，激光关）+ N14 单独补光
    if (scannerSerial && scannerSerial->isOpen()) {
        sendMcu("N14 B60;");
        QThread::msleep(200);
        sendMcu("N10 H50 B60 T1 V2 L0;");
        QThread::msleep(300);
        statusBar()->showMessage(QStringLiteral("补光灯已开 N14 B60 + N10 启动"));
    } else {
        statusBar()->showMessage(QStringLiteral("串口未打开，补光灯不可用"));
    }
    QApplication::processEvents();

    // ===== 逐姿态采集（25个姿态 × 每姿态5帧）=====
    struct CollectedPose {
        cv::Mat markerL, markerR;
        cv::Mat laserL[4], laserR[4];
        double temperature = 25.0;
    };
    std::vector<CollectedPose> collectedPoses;
    collectedPoses.reserve(25);

    const int totalPoses = 25;
    const char* laserNames[] = {"左斜激光", "右斜激光", "细节", "深孔"};
    cv::Size patternSize(calibration::CHESSBOARD_COLS - 1, calibration::CHESSBOARD_ROWS - 1);

    bool aborted = false;

    for (int poseIdx = 0; poseIdx < totalPoses; ++poseIdx) {
        progress.setValue(poseIdx);

        // 等待用户准备好（显示实时画面，按"继续"采集）
        updatePreview({}, {},
            QStringLiteral("姿态 %1/%2：请将标定板放到目标位置，准备就绪后等待自动采集...")
                .arg(poseIdx + 1).arg(totalPoses));

        // 倒计时3秒后自动采集
        for (int cd = 3; cd > 0; --cd) {
            if (progress.wasCanceled()) { aborted = true; break; }
            QApplication::processEvents();

            Scanner::hal::StereoFrame sf;
            if (cam->grabFrame(sf, 2000).success && !sf.leftGray.empty()) {
                updatePreview(sf.leftGray, sf.rightGray,
                    QStringLiteral("姿态 %1/%2：%3 秒后开始采集...")
                        .arg(poseIdx + 1).arg(totalPoses).arg(cd));
            }
            QThread::msleep(1000);
        }
        if (aborted) break;

        // ===== 直接采集5帧 =====
        CollectedPose cp;
        cp.temperature = cam->getTemperature();

        // 帧1：标志点帧（补光灯开，激光关）
        updatePreview({}, {},
            QStringLiteral("姿态 %1/%2 - 帧 1/5：标志点...").arg(poseIdx + 1).arg(totalPoses));
        sendMcu("N14 B60;");
        sendMcu("N13 L0;");
        QThread::msleep(150);
        {
            Scanner::hal::StereoFrame sf;
            if (cam->grabFrame(sf, 5000).success) {
                cp.markerL = sf.leftGray.clone();
                cp.markerR = sf.rightGray.clone();
                updatePreview(sf.leftGray, sf.rightGray,
                    QStringLiteral("姿态 %1 - 帧 1/5 标志点 OK").arg(poseIdx + 1));
            }
            QThread::msleep(300);
        }

        // 帧2-5：4种激光帧
        for (int laserIdx = 0; laserIdx < 4; ++laserIdx) {
            if (progress.wasCanceled()) { aborted = true; break; }
            updatePreview({}, {},
                QStringLiteral("姿态 %1/%2 - 帧 %3/5：%4...")
                    .arg(poseIdx + 1).arg(totalPoses).arg(laserIdx + 2)
                    .arg(QString::fromUtf8(laserNames[laserIdx])));
            sendMcu("N14 B0;");
            sendMcu(QString("N13 L%1;").arg(60 + laserIdx * 20));
            QThread::msleep(150);
            Scanner::hal::StereoFrame sf;
            if (cam->grabFrame(sf, 5000).success) {
                cp.laserL[laserIdx] = sf.leftGray.clone();
                cp.laserR[laserIdx] = sf.rightGray.clone();
                updatePreview(sf.leftGray, sf.rightGray,
                    QStringLiteral("姿态 %1 - 帧 %2/5 %3 OK")
                        .arg(poseIdx + 1).arg(laserIdx + 2).arg(QString::fromUtf8(laserNames[laserIdx])));
            }
            QThread::msleep(300);
        }
        if (aborted) break;

        // 恢复补光灯
        sendMcu("N14 B60;");
        sendMcu("N13 L0;");
        collectedPoses.push_back(std::move(cp));

        updatePreview({}, {},
            QStringLiteral("姿态 %1/%2 完成！5帧已采集，请移动到下一姿态...")
                .arg(poseIdx + 1).arg(totalPoses));
        QThread::msleep(1000);
    }

    progress.setValue(25);
    previewDlg.close();
    if (scannerSerial) { sendMcu("N11 H0;"); }

    if (aborted) {
        cam->stopCapture();
        return;
    }

    // 停止采集
    cam->stopCapture();

    if (collectedPoses.size() < 5) {
        QMessageBox::warning(this, QStringLiteral("标定失败"),
            QStringLiteral("采集姿态数不足: %1").arg(collectedPoses.size()));
        return;
    }

    // ===== 阶段3：相机标定（内参→外参→立体矫正→温度补偿表）=====
    statusBar()->showMessage(QStringLiteral("正在执行相机标定..."));
    QApplication::processEvents();

    calibration::CameraCalibInput camInput;
    camInput.imageWidth = 2048;
    camInput.imageHeight = 1536;
    camInput.cte = 23.6e-6;
    camInput.referenceTemp = 25.0;
    camInput.tempStep = 2.0;
    camInput.tempRangeMin = -10.0;
    camInput.tempRangeMax = 10.0;

    cv::Size cbPatternSize(calibration::CHESSBOARD_COLS - 1, calibration::CHESSBOARD_ROWS - 1);
    for (const auto& cp : collectedPoses) {
        if (cp.markerL.empty()) continue;
        std::vector<cv::Point2f> cornersL, cornersR;
        if (cv::findChessboardCorners(cp.markerL, cbPatternSize, cornersL,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE) &&
            cv::findChessboardCorners(cp.markerR, patternSize, cornersR,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE))
        {
            cv::cornerSubPix(cp.markerL, cornersL, cv::Size(5, 5), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.01));
            cv::cornerSubPix(cp.markerR, cornersR, cv::Size(5, 5), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.01));
            camInput.leftCorners.push_back(cornersL);
            camInput.rightCorners.push_back(cornersR);
        }
    }

    auto camResult = calibration::runCameraCalibration(camInput,
        [this](int pct, const std::string& step) {
            statusBar()->showMessage(QString::fromStdString(step) + " (" + QString::number(pct) + "%)");
            QApplication::processEvents();
        });

    if (!camResult.success) {
        QMessageBox::warning(this, QStringLiteral("标定失败"),
            QStringLiteral("相机标定失败: %1").arg(QString::fromStdString(camResult.message)));
        return;
    }
    calibration::saveCalibResult("calibration.json", camResult);

    // ===== 阶段4：激光器标定 =====
    statusBar()->showMessage(QStringLiteral("正在执行激光器标定..."));
    QApplication::processEvents();

    calibration::LaserCalibInput laserInput;
    laserInput.cameraCalib = &camResult;
    laserInput.initialTx = 80.0;
    laserInput.initialTy = 3.0;
    laserInput.initialTz = 3.0;
    laserInput.cte = 23.6e-6;
    laserInput.referenceTemp = 25.0;

    for (const auto& cp : collectedPoses) {
        // 每种激光管帧作为独立姿态输入
        for (int li = 0; li < 4; ++li) {
            if (!cp.laserL[li].empty() && !cp.laserR[li].empty()) {
                calibration::LaserPoseImages lpi;
                lpi.leftLaserGray = cp.laserL[li];
                lpi.rightLaserGray = cp.laserR[li];
                lpi.temperature = cp.temperature;
                laserInput.poses.push_back(std::move(lpi));
            }
        }
    }

    auto laserResult = calibration::runLaserCalibration(laserInput,
        [this](int pct, const std::string& step) {
            statusBar()->showMessage(QString::fromStdString(step) + " (" + QString::number(pct) + "%)");
            QApplication::processEvents();
        });

    if (laserResult.success)
        calibration::saveLaserCalibResult("laser_calib.json", laserResult);

    // 存到成员
    m_lastCameraCalib.success = true;
    m_lastCameraCalib.cameraMatrixL = camResult.cameraMatrixL;
    m_lastCameraCalib.distCoeffsL = camResult.distCoeffsL;
    m_lastCameraCalib.cameraMatrixR = camResult.cameraMatrixR;
    m_lastCameraCalib.distCoeffsR = camResult.distCoeffsR;
    m_lastCameraCalib.R = camResult.R;
    m_lastCameraCalib.T = camResult.T;
    m_lastCameraCalib.R1 = camResult.R1;
    m_lastCameraCalib.R2 = camResult.R2;
    m_lastCameraCalib.P1 = camResult.P1;
    m_lastCameraCalib.P2 = camResult.P2;
    m_lastCameraCalib.Q = camResult.Q;
    m_lastCameraCalib.intrinsicRMS = camResult.intrinsicRMS;
    m_lastCameraCalib.stereoReprojError = camResult.stereoReprojError;
    m_lastCameraCalib.hasTempTables = camResult.hasTempTables;

    // ===== 汇总标定误差 =====
    QString summary = QStringLiteral(
        "====== 标定完成 ======\n\n"
        "【采集统计】\n"
        "  目标姿态数: %1\n"
        "  已采集姿态: %2\n"
        "  激光帧数: %3\n\n"
        "【相机标定】\n"
        "  内参 RMS: %4 px\n"
        "  立体重投影误差: %5 px\n"
        "  有效帧数: %6\n"
        "  温度补偿表: %7\n\n")
        .arg(totalPoses)
        .arg(collectedPoses.size())
        .arg(laserInput.poses.size())
        .arg(camResult.intrinsicRMS, 0, 'f', 4)
        .arg(camResult.stereoReprojError, 0, 'f', 4)
        .arg(camResult.validFrameCount)
        .arg(camResult.hasTempTables ? QStringLiteral("已生成") : QStringLiteral("未生成"));

    if (laserResult.success) {
        summary += QStringLiteral(
            "【激光器标定】\n"
            "  投影机光心 T: (%1, %2, %3) mm\n"
            "  最终 RMS: %4\n"
            "  优化改善率: %5\n"
            "  姿态数: %6\n"
            "  总点数: %7\n")
            .arg(laserResult.projectorT[0], 0, 'f', 2)
            .arg(laserResult.projectorT[1], 0, 'f', 2)
            .arg(laserResult.projectorT[2], 0, 'f', 2)
            .arg(laserResult.finalRms, 0, 'f', 4)
            .arg(laserResult.improvementRatio, 0, 'f', 2)
            .arg(laserResult.poseCount)
            .arg(laserResult.totalPointCount);
    } else {
        summary += QStringLiteral("\n【激光器标定】失败: %1\n")
            .arg(QString::fromStdString(laserResult.message));
    }

    summary += QStringLiteral("\n标定参数已保存: calibration.json / laser_calib.json");

    QMessageBox::information(this, QStringLiteral("标定结果"), summary);
    statusBar()->showMessage(QStringLiteral("标定完成"));
}

void MainWindow::onScanClicked()
{
    // 切换回默认界面：隐藏标定板和彩条，恢复相机
    if (m_calibBoard2D) m_calibBoard2D->hide();
    // 隐藏左右、前后彩条
    auto* viewArea = m_3dView ? m_3dView->parentWidget()->parentWidget() : nullptr;
    if (viewArea) {
        auto* lrBar = viewArea->findChild<QWidget*>("calibLrBar");
        if (lrBar) lrBar->hide();
    }
    auto* viewContainer = m_3dView ? m_3dView->parentWidget() : nullptr;
    if (viewContainer) {
        auto* fbBar = viewContainer->findChild<QWidget*>("calibFbBar");
        if (fbBar) fbBar->hide();
    }
    // 清空 3D 场景，恢复初始状态
    if (m_3dView) {
        m_3dView->clearScene();
        m_3dView->setCenterOverlayVisible(true);
        m_3dView->viewer()->getCamera()->setClearColor(osg::Vec4(0.412f, 0.412f, 0.412f, 1.0f));
        auto* manip = new osgGA::TrackballManipulator();
        m_3dView->setCameraManipulator(manip);
        manip->home(0);
    }
    // 显示悬浮工具条
    if (m_floatingToolbar) {
        m_floatingToolbar->setVisible(true);
        m_floatingToolbar->show();
    }

    auto* series = static_cast<LEADSCANSeries*>(m_integrateTestDialog);
    if (!series) {
        if (!m_integrateTestDialog) m_integrateTestDialog = new LEADSCANSeries();
        series = static_cast<LEADSCANSeries*>(m_integrateTestDialog);
    }
    auto* cam = series->getCameraControl();
    if (!cam || !cam->isScannerCameraOpen()) {
        QMessageBox::warning(this, QStringLiteral("扫描"), QStringLiteral("请先在集成测试中打开扫描相机"));
        return;
    }

    statusBar()->showMessage(QStringLiteral("扫描中..."));
    QApplication::processEvents();

    calibration::ScanWorkflow workflow;
    workflow.setProgressCallback([this](int pct, const std::string& step) {
        statusBar()->showMessage(QString::fromStdString(step) + " (" + QString::number(pct) + "%)");
        QApplication::processEvents();
    });

    calibration::ScanInput input;

    std::vector<cv::Point3f> fusedCloud, fusedNormals;
    auto result = workflow.processFrameToCloud(input, fusedCloud, fusedNormals);

    if (result.success && m_3dView && !fusedCloud.empty()) {
        std::vector<osg::Vec3> osgPoints;
        osgPoints.reserve(fusedCloud.size());
        for (const auto& p : fusedCloud) osgPoints.emplace_back(p.x, p.y, p.z);
        m_3dView->loadPointCloud(osgPoints);
        QMessageBox::information(this, QStringLiteral("扫描完成"),
            QStringLiteral("点云: %1 点\n法线: %2\n耗时: %3 ms")
            .arg(result.fusedPoints).arg(result.fusedPoints).arg(static_cast<int>(result.totalTimeMs)));
    } else {
        QMessageBox::warning(this, QStringLiteral("扫描"), QString::fromStdString(result.message));
    }
}

// ============================================================================
// 三种扫描模式
// ============================================================================

void MainWindow::onScanMarkers() {
    appendDebugLog(QStringLiteral(">>> 点击「标点扫描」"));
    startScanWithConfig(Scanner::workflow::ScanConfig::ModeA(), 2);
}

void MainWindow::onScanMesh() {
    appendDebugLog(QStringLiteral(">>> 点击「面片扫描」"));
    startScanWithConfig(Scanner::workflow::ScanConfig::ModeB(), 3);
}

void MainWindow::onScanPointCloud() {
    appendDebugLog(QStringLiteral(">>> 点击「点云扫描」"));
    if (m_importedMarkers.empty()) {
        QMessageBox::warning(this, QStringLiteral("点云扫描"),
            QStringLiteral("请先在「文件管理」中导入全局标志点"));
        appendDebugLog(QStringLiteral("⚠ 未导入全局标志点，无法进行点云扫描"));
        return;
    }
    auto config = Scanner::workflow::ScanConfig::ModeC();
    // 导入标志点 → initialGlobalMarkers
    config.initialGlobalMarkers.reserve(m_importedMarkers.size());
    for (const auto& m : m_importedMarkers) {
        calib::MarkerFuseInput mi;
        mi.x = m.x(); mi.y = m.y(); mi.z = m.z();
        mi.nx = 0; mi.ny = 0; mi.nz = 1;
        mi.whiteRadius = 0;
        config.initialGlobalMarkers.push_back(mi);
    }
    appendDebugLog(QStringLiteral("已加载 %1 个全局标志点").arg(m_importedMarkers.size()));
    startScanWithConfig(config, 4);
}

void MainWindow::onStopScan() {
    stopScan();
}

void MainWindow::stopScan() {
    if (!m_scanning) return;
    appendDebugLog(QStringLiteral("<<< 停止扫描 (模式=%1)").arg(m_scanModeIdx));
    m_scanning = false;
    if (m_scanThread.joinable()) m_scanThread.join();
    m_scanModeIdx = -1;

    // 不停相机采集（避免反复 stop/start 导致 Galaxy SDK 崩溃）
    // 相机保持连续采集，只停扫描处理线程

    auto* scannerSerial = m_appCtx ? m_appCtx->scannerSerial() : nullptr;
    if (scannerSerial && scannerSerial->isOpen()) {
        scannerSerial->send("N11 H0;");
        appendDebugLog(QStringLiteral("串口发送: N11 H0; (停止扫描)"));
        scannerSerial->send("N14 B0;");
        appendDebugLog(QStringLiteral("串口发送: N14 B0; (补光灯关)"));
        scannerSerial->send("N13 L0;");
        appendDebugLog(QStringLiteral("串口发送: N13 L0; (激光关)"));
    }

    statusBar()->showMessage(QStringLiteral("扫描已停止"));
}

void MainWindow::startScanWithConfig(const Scanner::workflow::ScanConfig& config, int modeIdx) {
    // 正在扫描：先停
    if (m_scanning) {
        stopScan();
    }

    auto* cam = m_appCtx ? m_appCtx->camera() : nullptr;
    if (!cam || !cam->isOpen()) {
        QMessageBox::warning(this, QStringLiteral("扫描"), QStringLiteral("相机未连接"));
        return;
    }

    // 切换到扫描视图
    if (m_calibBoard2D) m_calibBoard2D->hide();
    if (m_3dView) {
        m_3dView->clearScene();
        m_3dView->setCenterOverlayVisible(true);
        m_3dView->viewer()->getCamera()->setClearColor(osg::Vec4(0.412f, 0.412f, 0.412f, 1.0f));
        auto* manip = new osgGA::TrackballManipulator();
        m_3dView->setCameraManipulator(manip);
        manip->home(0);
    }
    if (m_floatingToolbar) { m_floatingToolbar->setVisible(true); m_floatingToolbar->show(); }

    // 创建并初始化扫描流水线
    if (!m_scanPipeline) {
        m_scanPipeline = std::make_unique<Scanner::workflow::ScanPipeline>(config);
    } else {
        m_scanPipeline->reset();
    }

    // 加载标定参数（如果有）
    if (m_lastCameraCalib.success) {
        Scanner::workflow::ScanCalibration calib;
        calib.cameraMatrixL = m_lastCameraCalib.cameraMatrixL;
        calib.distCoeffsL = m_lastCameraCalib.distCoeffsL;
        calib.cameraMatrixR = m_lastCameraCalib.cameraMatrixR;
        calib.distCoeffsR = m_lastCameraCalib.distCoeffsR;
        calib.R1 = m_lastCameraCalib.R1;
        calib.R2 = m_lastCameraCalib.R2;
        calib.P1 = m_lastCameraCalib.P1;
        calib.P2 = m_lastCameraCalib.P2;
        calib.Q = m_lastCameraCalib.Q;
        calib.imageSize = cv::Size(2048, 1536);
        calib.valid = true;
        m_scanPipeline->setCalibration(calib);
        spdlog::info("[Scan] 标定参数已加载");
    } else {
        spdlog::warn("[Scan] 无标定参数，3D重建将跳过");
    }

    // 设置进度回调
    m_scanPipeline->setProgressCallback([this](const Scanner::workflow::ScanProgress& p) {
        QMetaObject::invokeMethod(this, [this, p]() {
            statusBar()->showMessage(QStringLiteral("扫描中: %1帧 | 标记点 %2 | 融合 %3 | %4")
                .arg(p.frameCount).arg(p.markerCount).arg(p.fusedPointCount)
                .arg(QString::fromStdString(p.status)));
        });
    });

    if (!m_scanPipeline->initialize()) {
        QMessageBox::warning(this, QStringLiteral("扫描"), QStringLiteral("流水线初始化失败"));
        return;
    }

    // 先发串口命令（补光灯+激光），再启动相机采集，避免相机先拍黑帧
    auto* scannerSerial = m_appCtx ? m_appCtx->scannerSerial() : nullptr;
    if (scannerSerial && scannerSerial->isOpen()) {
        appendDebugLog(QStringLiteral("串口 %1 已打开").arg(scannerSerial->portName()));
        QString startCmd = config.enableLaser ?
            QStringLiteral("N10 H50 B60 T1 V2 L60;") : QStringLiteral("N10 H50 B60 T1 V2 L0;");
        scannerSerial->send(startCmd);
        appendDebugLog(QStringLiteral("串口发送: %1 (激光=%2)").arg(startCmd).arg(config.enableLaser ? "开" : "关"));
    } else {
        appendDebugLog(QStringLiteral("⚠ 串口未打开！补光灯不可用"));
    }

    // 确保相机在连续采集模式
    if (!cam->isCapturing()) {
        statusBar()->showMessage(QStringLiteral("启动相机连续采集..."));
        QApplication::processEvents();
        try {
            cam->setExposure(5.0);
            auto r = cam->startAsyncCaptureContinuous([](const Scanner::hal::StereoFrame&) {});
            spdlog::info("[Scan] startAsyncCaptureContinuous success={}", r.success);
            if (!r.success) {
                QMessageBox::warning(this, QStringLiteral("扫描"),
                    QStringLiteral("相机采集启动失败: %1").arg(QString::fromStdString(r.message)));
                return;
            }
        } catch (const std::exception& e) {
            spdlog::error("[Scan] 相机启动异常: {}", e.what());
            QMessageBox::warning(this, QStringLiteral("扫描"),
                QStringLiteral("相机启动异常: %1").arg(QString::fromUtf8(e.what())));
            return;
        }
    }

    // 启动扫描线程
    m_scanning = true;
    m_scanModeIdx = modeIdx;
    statusBar()->showMessage(QStringLiteral("扫描中... (再次点击同一按钮停止)"));
    m_scanThread = std::thread(&MainWindow::scanLoop, this);
}

void MainWindow::scanLoop() {
    spdlog::info("[Scan] 扫描线程启动");
    auto* cam = m_appCtx ? m_appCtx->camera() : nullptr;
    int frameCount = 0;

    while (m_scanning && cam) {
        try {
            Scanner::hal::StereoFrame sf;
            auto gr = cam->grabFrame(sf, 3000);
            if (!gr.success || sf.leftGray.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }

            ++frameCount;
            auto output = m_scanPipeline->processFrame(sf.leftGray, sf.rightGray);

            // 定期更新3D视图（每30帧）
            if (frameCount % 30 == 0 && m_3dView) {
                const auto& markers = m_scanPipeline->getFusedMarkers();
                if (!markers.empty()) {
                    std::vector<osg::Vec3> pts;
                    pts.reserve(markers.size());
                    for (const auto& m : markers) {
                        pts.emplace_back(m.x, m.y, m.z);
                    }
                    QMetaObject::invokeMethod(this, [this, pts]() {
                        if (m_3dView) m_3dView->loadMarkerPoints(pts);
                    });
                }
            }

            if (frameCount % 100 == 0) {
                appendDebugLog(QStringLiteral("已处理 %1 帧").arg(frameCount));
            }
        } catch (const std::exception& e) {
            spdlog::error("[Scan] 帧 {} 异常: {}", frameCount, e.what());
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } catch (...) {
            spdlog::error("[Scan] 帧 {} 未知异常", frameCount);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    spdlog::info("[Scan] 扫描线程结束 ({}帧)", frameCount);

    // 最终更新点云
    if (m_3dView) {
        const auto& markers = m_scanPipeline->getFusedMarkers();
        if (!markers.empty()) {
            std::vector<osg::Vec3> pts;
            pts.reserve(markers.size());
            for (const auto& m : markers) {
                pts.emplace_back(m.x, m.y, m.z);
            }
            QMetaObject::invokeMethod(this, [this, pts]() {
                if (m_3dView) m_3dView->loadMarkerPoints(pts);
            });
        }
    }

    QMetaObject::invokeMethod(this, [this]() {
        statusBar()->showMessage(QStringLiteral("扫描完成"));
    });
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    QMessageBox msgBox(this);
    msgBox.setWindowTitle(QStringLiteral("确认关闭"));
    msgBox.setText(QStringLiteral("确认关闭扫描仪软件？"));
    QPushButton* btnConfirm = msgBox.addButton(QStringLiteral("确认"), QMessageBox::YesRole);
    msgBox.addButton(QStringLiteral("取消"), QMessageBox::NoRole);
    msgBox.exec();
    if (msgBox.clickedButton() != btnConfirm) {
        event->ignore();
        return;
    }
    spdlog::info("[MainWindow] 正在关闭...");

    // 1. 停止扫描线程
    m_scanning = false;
    if (m_scanThread.joinable()) m_scanThread.join();
    spdlog::info("[MainWindow] 扫描线程已停止");

    // 2. 销毁扫描流水线
    m_scanPipeline.reset();

    // 3. 关闭相机（停止采集 + 关闭设备）
    if (m_appCtx && m_appCtx->camera()) {
        auto* cam = m_appCtx->camera();
        if (cam->isCapturing()) {
            cam->stopCapture();
            spdlog::info("[MainWindow] 相机采集已停止");
        }
        if (cam->isOpen()) {
            cam->close();
            spdlog::info("[MainWindow] 相机已关闭");
        }
    }

    // 4. 断开串口（关补光灯/激光 + 关闭）
    if (m_appCtx && m_appCtx->scannerSerial()) {
        auto* serial = m_appCtx->scannerSerial();
        if (serial->isOpen()) {
            serial->send("N11 H0;");  // 停止扫描
            serial->send("N14 B0;");  // 补光灯关
            serial->send("N13 L0;");  // 激光关
            serial->close();
            spdlog::info("[MainWindow] 串口已断开");
        }
    }

    // 5. 关闭子窗口
    if (m_floatingToolbar) {
        m_floatingToolbar->close();
        delete m_floatingToolbar;
        m_floatingToolbar = nullptr;
    }
    if (m_integrateTestDialog) {
        m_integrateTestDialog->close();
        delete m_integrateTestDialog;
        m_integrateTestDialog = nullptr;
    }
    if (m_calibDialog) {
        m_calibDialog->close();
        delete m_calibDialog;
        m_calibDialog = nullptr;
    }

    // 6. AppContext 清理（关闭硬件监控等）
    if (m_appCtx) {
        m_appCtx->shutdown();
    }

    spdlog::info("[MainWindow] 关闭完成");
    QMainWindow::closeEvent(event);
}

void MainWindow::createFloatingToolbar()
{
    m_floatingToolbar = new QWidget();
    m_floatingToolbar->setObjectName("floatingToolbar");
    m_floatingToolbar->setWindowFlags(Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint);
    m_floatingToolbar->setAttribute(Qt::WA_TranslucentBackground);
    m_floatingToolbar->setStyleSheet("background-color: white; border: 1px solid #e0e0e0; border-radius: 8px;");
    m_floatingToolbar->setMinimumWidth(400);
    m_floatingToolbar->setMaximumWidth(800);
    m_floatingToolbar->setFixedHeight(47);

    QHBoxLayout *toolbarLayout = new QHBoxLayout(m_floatingToolbar);
    toolbarLayout->setContentsMargins(16, 0, 16, 0);
    toolbarLayout->setSpacing(8);
    toolbarLayout->addStretch();

    toolbarLayout->addWidget(createBottomToolBar());
    toolbarLayout->addStretch();

    m_floatingToolbar->installEventFilter(this);
    m_floatingToolbar->adjustSize();
    m_floatingToolbar->show();
    QTimer::singleShot(100, this, [this]() { repositionFloatingToolbar(); });
}

void MainWindow::repositionFloatingToolbar()
{
    if (!m_floatingToolbar || !m_3dViewArea) return;
    if (!m_floatingToolbar->isVisible()) return;
    m_floatingToolbar->adjustSize();
    // 基于整个显示窗口(view3DArea)居中，而不是只基于 m_3dView
    QRect areaGeo = m_3dViewArea->geometry();
    QPoint areaBottomCenter = m_3dViewArea->mapToGlobal(QPoint(areaGeo.width() / 2, areaGeo.height()));
    int tbX = areaBottomCenter.x() - m_floatingToolbar->width() / 2;
    int tbY = areaBottomCenter.y() - m_floatingToolbar->height() - 10;
    m_floatingToolbar->move(tbX, tbY);
}

void MainWindow::resizeEvent(QResizeEvent *event)
{
    QMainWindow::resizeEvent(event);
    QTimer::singleShot(0, this, [this]() { repositionFloatingToolbar(); });
}

QWidget *MainWindow::createTitleBar()
{
    QWidget *bar = new QWidget();
    bar->setObjectName("titleBar");
    bar->setMinimumHeight(28);
    bar->setMaximumHeight(36);
    QHBoxLayout *layout = new QHBoxLayout(bar);
    layout->setContentsMargins(8, 0, 0, 0);
    layout->setSpacing(0);

    QPushButton *btnLogo = new QPushButton();
    btnLogo->setObjectName("btnLogo");
    btnLogo->setFixedSize(28, 28);
    btnLogo->setIcon(QIcon(renderSvg(":/icons/resources/icons/icon/firstandsecond/trace-black-11.svg", 18)));
    btnLogo->setIconSize(QSize(18, 18));
    layout->addWidget(btnLogo);

    QPushButton *btnPrev = new QPushButton();
    btnPrev->setObjectName("btnTitleAction");
    btnPrev->setFixedSize(28, 28);
    btnPrev->setIcon(QIcon(renderSvg(":/icons/resources/icons/icon/firstandsecond/save-red-13.svg", 14)));
    btnPrev->setIconSize(QSize(14, 14));
    layout->addWidget(btnPrev);

    QPushButton *btnLast = new QPushButton();
    btnLast->setObjectName("btnTitleAction");
    btnLast->setFixedSize(28, 28);
    btnLast->setIcon(QIcon(renderSvg(":/icons/resources/icons/icon/firstandsecond/last-red-13.svg", 14)));
    btnLast->setIconSize(QSize(14, 14));
    layout->addWidget(btnLast);

    QPushButton* btnNext = new QPushButton();
    btnNext->setObjectName("btnTitleAction");
    btnNext->setFixedSize(28, 28);
    btnNext->setIcon(QIcon(renderSvg(":/icons/resources/icons/icon/firstandsecond/next-red-13.svg", 14)));
    btnNext->setIconSize(QSize(14, 14));
    layout->addWidget(btnNext);

    layout->addStretch();

    m_projectName = new QLabel(QStringLiteral("工程001_Turbine_Blade - V2.0.4 PRO"));
    m_projectName->setObjectName("projectNameLabel");
    m_projectName->setAlignment(Qt::AlignCenter);
    layout->addWidget(m_projectName);

    layout->addStretch();

    QPushButton *btnSave = new QPushButton();
    btnSave->setObjectName("btnTitleAction");
    btnSave->setFixedSize(40, 36);
    btnSave->setIcon(QIcon(renderSvg(":/icons/resources/icons/保存-黑-13.svg", 14)));
    btnSave->setIconSize(QSize(14, 14));
    layout->addWidget(btnSave);

    QPushButton *btnMin = new QPushButton();
    btnMin->setObjectName("btnWindowControl");
    btnMin->setFixedSize(40, 36);
    btnMin->setText(QStringLiteral("--"));
    layout->addWidget(btnMin);

    QPushButton *btnClose = new QPushButton();
    btnClose->setObjectName("btnWindowControl");
    btnClose->setFixedSize(40, 36);
    btnClose->setText("X");
    connect(btnClose, &QPushButton::clicked, this, &QWidget::close);
    layout->addWidget(btnClose);

    QString titleBtnStyle = "QPushButton { background-color: transparent; border: none; }"
                            "QPushButton:hover { background-color: rgba(0,0,0,0.05); }";
    for (int i = 0; i < layout->count(); ++i) {
        QPushButton *btn = qobject_cast<QPushButton*>(layout->itemAt(i)->widget());
        if (btn) btn->setStyleSheet(titleBtnStyle);
    }

    return bar;
}

QWidget *MainWindow::createNavBar()
{
    QWidget *bar = new QWidget();
    bar->setObjectName("navBar");
    bar->setMinimumHeight(32);
    bar->setMaximumHeight(42);
    bar->setStyleSheet("background-color: #8B1A2B; color: white;");
    QHBoxLayout *layout = new QHBoxLayout(bar);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);

    QWidget *leftGroup = new QWidget();
    leftGroup->setMinimumWidth(200);
    leftGroup->setStyleSheet("border: none;");
    QHBoxLayout *leftLayout = new QHBoxLayout(leftGroup);
    leftLayout->setContentsMargins(0, 0, 0, 0);
    leftLayout->setSpacing(0);

    QPushButton *btnFile = new QPushButton(QStringLiteral("菜单"));
    btnFile->setObjectName("navButton");
    btnFile->setFixedHeight(42);
    btnFile->setFixedWidth(50);
    leftLayout->addWidget(btnFile);
    m_navLeftButtons.append(btnFile);

    QPushButton *btnScan = new QPushButton(QStringLiteral("扫描"));
    btnScan->setObjectName("navButtonActive");
    btnScan->setFixedHeight(42);
    btnScan->setFixedWidth(50);
    leftLayout->addWidget(btnScan);
    m_navLeftButtons.append(btnScan);
    connect(btnScan, &QPushButton::clicked, this, &MainWindow::onScanClicked);

    QPushButton *btnManage = new QPushButton(QStringLiteral("管理"));
    btnManage->setObjectName("navButton");
    btnManage->setFixedHeight(42);
    btnManage->setFixedWidth(50);
    leftLayout->addWidget(btnManage);
    m_navLeftButtons.append(btnManage);

    QPushButton *btnIntegrateTest = new QPushButton(QStringLiteral("集成测试"));
    btnIntegrateTest->setObjectName("navButton");
    btnIntegrateTest->setFixedHeight(42);
    btnIntegrateTest->setFixedWidth(75);
    leftLayout->addWidget(btnIntegrateTest);
    m_navLeftButtons.append(btnIntegrateTest);
    connect(btnIntegrateTest, &QPushButton::clicked, this, &MainWindow::onIntegrateTestClicked);

    QPushButton *btnReloadCloud = new QPushButton(QStringLiteral("加载点云"));
    btnReloadCloud->setObjectName("navButton");
    btnReloadCloud->setFixedHeight(42);
    btnReloadCloud->setFixedWidth(75);
    leftLayout->addWidget(btnReloadCloud);
    m_navLeftButtons.append(btnReloadCloud);
    connect(btnReloadCloud, &QPushButton::clicked, this, &MainWindow::onReloadPointCloud);

    layout->addWidget(leftGroup);
    layout->addStretch();

    QWidget *rightGroup = new QWidget();
    rightGroup->setMinimumWidth(300);
    rightGroup->setStyleSheet("border: none;");
    QHBoxLayout *rightLayout = new QHBoxLayout(rightGroup);
    rightLayout->setContentsMargins(-10, 0, 0, 0);
    rightLayout->setSpacing(0);

    QStringList scanModes = {
        QStringLiteral("手持扫描"), QStringLiteral("跟踪扫描"),
        QStringLiteral("摄影测量"), QStringLiteral("自动扫描"),
        QStringLiteral("检测分析")
    };

    QStringList scanModeIcons = {
        "handlescan-white-11", "trace-white-11",
        "photo-white-11", "auto-white-11",
        "analysis-white-11"
    };

    for (int i = 0; i < scanModes.size(); ++i) {
        QPushButton *btn = new QPushButton();
        btn->setObjectName("scanModeButton");
        btn->setFixedHeight(42);
        btn->setFixedWidth(84);

        QHBoxLayout *btnLayout = new QHBoxLayout(btn);
        btnLayout->setContentsMargins(6, 0, 6, 0);
        btnLayout->setSpacing(4);
        btnLayout->setAlignment(Qt::AlignCenter);

        QLabel *iconLbl = new QLabel();
        iconLbl->setStyleSheet("border: none; background: transparent;");
        int iconSize = 14;
        iconLbl->setPixmap(renderSvg(QString(":/icons/resources/icons/icon/firstandsecond/%1.svg").arg(scanModeIcons[i]), iconSize));
        iconLbl->setFixedSize(iconSize, iconSize);
        btnLayout->addWidget(iconLbl);

        QLabel *textLbl = new QLabel(scanModes[i]);
        textLbl->setObjectName("scanModeText");
        textLbl->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
        btnLayout->addWidget(textLbl);

        rightLayout->addWidget(btn);
        m_navRightButtons.append(btn);
    }

    layout->addWidget(rightGroup);
    return bar;
}

QWidget *MainWindow::createToolBar()
{
    QWidget *bar = new QWidget();
    bar->setObjectName("toolBar");
    bar->setMinimumHeight(40);
    bar->setMaximumHeight(56);
    QHBoxLayout *layout = new QHBoxLayout(bar);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(32);

    struct ToolItem { QString icon; QString text; };
    QList<ToolItem> items = {
        {"filemanager-black-14", QStringLiteral("文件管理")},
        {"equipcalib-black-14", QStringLiteral("校准设备")},
        {"markscan-black-14", QStringLiteral("标点扫描")},
        {"meshscan-black-14", QStringLiteral("面片扫描")},
        {"cloudscan-black-14", QStringLiteral("点云扫描")},
        {"both-black-14", QStringLiteral("正反扫描")},
        {"slicerscan-black-14", QStringLiteral("切面扫描")},
        {"projmanager-black-14", QStringLiteral("重置项目")}
    };

    for (int i = 0; i < items.size(); ++i) {
        QPushButton *btn = new QPushButton();
        btn->setObjectName("toolButton");
        btn->setFixedSize(64, 56);
        QVBoxLayout *btnLayout = new QVBoxLayout(btn);
        btnLayout->setContentsMargins(0, 4, 0, 4);
        btnLayout->setSpacing(2);
        btnLayout->setAlignment(Qt::AlignCenter);

        QLabel *iconLbl = new QLabel();
        int iconSize = 20;
        iconLbl->setPixmap(renderSvg(QString(":/icons/resources/icons/icon/third/%1.svg").arg(items[i].icon), iconSize));
        iconLbl->setFixedSize(iconSize, iconSize);
        btnLayout->addWidget(iconLbl, 0, Qt::AlignHCenter);

        QLabel *textLbl = new QLabel(items[i].text);
        textLbl->setObjectName("toolButtonText");
        textLbl->setAlignment(Qt::AlignCenter);
        textLbl->setMinimumWidth(btn->width());
        btnLayout->addWidget(textLbl);

        if (i == 0) btn->setProperty("active", true);
        btn->setStyleSheet("QPushButton { background-color: transparent; border: none; }"
                           "QPushButton:hover { background-color: rgba(0,0,0,0.05); }");
        layout->addWidget(btn);
        m_toolButtons.append(btn);

        if (i == 1) {
            connect(btn, &QPushButton::clicked, this, &MainWindow::onCalibDeviceClicked);
        }

        // 文件管理：弹出导入/导出菜单
        if (i == 0) {
            connect(btn, &QPushButton::clicked, this, [this, btn]() {
                QMenu menu(btn);
                menu.setStyleSheet("QMenu { background: white; border: 1px solid #d0d0d0; }"
                                   "QMenu::item { padding: 6px 24px; }"
                                   "QMenu::item:selected { background: #e0e0e0; }");

                menu.addAction(QStringLiteral("导入标志点"), [this]() {
                    QString path = QFileDialog::getOpenFileName(this, QStringLiteral("导入标志点"), "", "Marker Files (*.json *.txt *.ply)");
                    if (path.isEmpty()) return;
                    std::string spath = path.toStdString();
                    std::vector<osg::Vec3> markers;
                    if (file_io::importMarkers(spath, markers)) {
                        m_importedMarkers = markers;
                        appendDebugLog(QStringLiteral("导入全局标志点 %1 个: %2").arg(markers.size()).arg(path));
                        statusBar()->showMessage(QStringLiteral("已导入 %1 个全局标志点，可进行点云扫描").arg(markers.size()));

                        // 在3D视图显示导入的标志点
                        if (m_3dView && !markers.empty()) {
                            m_3dView->clearScene();
                            m_3dView->loadMarkerPoints(markers);
                            m_3dView->setCenterOverlayVisible(true);
                            auto* manip = new osgGA::TrackballManipulator();
                            m_3dView->setCameraManipulator(manip);
                            manip->home(0);
                        }

                        QMessageBox::information(this, QStringLiteral("导入成功"),
                            QStringLiteral("已导入 %1 个全局标志点。\n点击「点云扫描」开始配准扫描。").arg(markers.size()));
                    } else {
                        statusBar()->showMessage(QStringLiteral("导入失败"));
                        QMessageBox::warning(this, QStringLiteral("导入失败"), QStringLiteral("无法解析标志点文件"));
                    }
                });
                menu.addAction(QStringLiteral("\xe5\xaf\xbc\xe5\x85\xa5\xe7\x82\xb9\xe4\xba\x91"), [this]() {
                    QString path = QFileDialog::getOpenFileName(this, QStringLiteral("\xe5\xaf\xbc\xe5\x85\xa5\xe7\x82\xb9\xe4\xba\x91"), "", "Point Cloud (*.ply *.pcd *.xyz *.txt);;All Files (*.*)");
                    if (path.isEmpty()) return;
                    QByteArray ba = path.toUtf8();
                    std::string spath(ba.constData(), ba.size());

                    QProgressDialog progress(QStringLiteral("Importing point cloud..."), QString(), 0, 0, this);
                    progress.setWindowModality(Qt::WindowModal);
                    progress.setMinimumDuration(0);
                    progress.setCancelButton(nullptr);
                    progress.setRange(0, 0);
                    progress.setAutoClose(false);
                    progress.setAutoReset(false);
                    progress.show();
                    QApplication::processEvents();

                    std::vector<osg::Vec3> points;
                    bool ok = file_io::importPointCloud(spath, points);

                    // 诊断日志写文件
                    FILE* logf = fopen("E:/workfold/framework/build/import_debug.log", "a");
                    if (logf) {
                        fprintf(logf, "=== import ===\n");
                        fprintf(logf, "file: %s\n", spath.c_str());
                        fprintf(logf, "ok=%d points=%zu\n", ok?1:0, points.size());
                        if (!points.empty()) {
                            float minx=1e30,miny=1e30,minz=1e30,maxx=-1e30,maxy=-1e30,maxz=-1e30;
                            for (const auto& p : points) {
                                if (p.x()<minx) minx=p.x(); if (p.x()>maxx) maxx=p.x();
                                if (p.y()<miny) miny=p.y(); if (p.y()>maxy) maxy=p.y();
                                if (p.z()<minz) minz=p.z(); if (p.z()>maxz) maxz=p.z();
                            }
                            fprintf(logf, "bbox: x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]\n",
                                    minx,maxx,miny,maxy,minz,maxz);
                            fprintf(logf, "first5: (%.2f,%.2f,%.2f) (%.2f,%.2f,%.2f) (%.2f,%.2f,%.2f) (%.2f,%.2f,%.2f) (%.2f,%.2f,%.2f)\n",
                                    points[0].x(),points[0].y(),points[0].z(),
                                    points[1].x(),points[1].y(),points[1].z(),
                                    points[2].x(),points[2].y(),points[2].z(),
                                    points[3].x(),points[3].y(),points[3].z(),
                                    points[4].x(),points[4].y(),points[4].z());
                        }
                        fprintf(logf, "m_3dView=%p\n", (void*)m_3dView);
                        fclose(logf);
                    }

                    if (ok && !points.empty()) {
                        statusBar()->showMessage(QStringLiteral("Imported %1 points").arg(points.size()));
                        if (m_3dView) {
                            m_3dView->setCenterOverlayVisible(false);
                            m_3dView->loadPointCloud(points);
                            m_3dView->update();
                        }
                    } else {
                        statusBar()->showMessage("Import failed");
                        QMessageBox::warning(this, "Import", "Failed to read point cloud. Check file format.");
                    }
                    progress.close();
                });
                menu.addAction(QStringLiteral("\xe5\xaf\xbc\xe5\x85\xa5\xe7\xbd\x91\xe6\xa0\xbc"), [this]() {
                    QString path = QFileDialog::getOpenFileName(this, QStringLiteral("\xe5\xaf\xbc\xe5\x85\xa5\xe7\xbd\x91\xe6\xa0\xbc"), "", "Mesh (*.stl *.obj);;All Files (*.*)");
                    if (path.isEmpty()) return;
                    if (m_3dView) {
                        QProgressDialog progress(QStringLiteral("Importing mesh..."), QString(), 0, 0, this);
                        progress.setWindowModality(Qt::WindowModal);
                        progress.setMinimumDuration(0);
                        progress.setCancelButton(nullptr);
                        progress.setRange(0, 0);
                        progress.setAutoClose(false);
                        progress.setAutoReset(false);
                        progress.show();
                        QApplication::processEvents();

                        m_3dView->clearScene();
                        if (m_3dView->loadMesh(path))
                            statusBar()->showMessage(QStringLiteral("\xe5\xaf\xbc\xe5\x85\xa5\xe7\xbd\x91\xe6\xa0\xbc\xe6\x88\x90\xe5\x8a\x9f: ") + path);
                        else {
                            statusBar()->showMessage(QStringLiteral("\xe5\xaf\xbc\xe5\x85\xa5\xe7\xbd\x91\xe6\xa0\xbc\xe5\xa4\xb1\xe8\xb4\xa5"));
                            QMessageBox::warning(this, "Import", "Failed to read mesh.");
                        }
                        m_3dView->update();
                        progress.close();
                    }
                });
                menu.addAction(QStringLiteral("\xe5\xaf\xbc\xe5\x85\xa5\xe5\xb7\xa5\xe7\xa8\x8b\xe6\x96\x87\xe4\xbb\xb6"), [this]() {
                    QString path = QFileDialog::getOpenFileName(this, QStringLiteral("\xe5\xaf\xbc\xe5\x85\xa5\xe5\xb7\xa5\xe7\xa8\x8b\xe6\x96\x87\xe4\xbb\xb6"), "", "Project (*.leadscan)");
                    if (!path.isEmpty()) statusBar()->showMessage(QStringLiteral("\xe5\xaf\xbc\xe5\x85\xa5: ") + path);
                });

                menu.addSeparator();

                menu.addAction(QStringLiteral("\xe5\xaf\xbc\xe5\x87\xba\xe6\xa0\x87\xe5\xbf\x97\xe7\x82\xb9"), [this]() {
                    QString path = QFileDialog::getSaveFileName(this, QStringLiteral("\xe5\xaf\xbc\xe5\x87\xba\xe6\xa0\x87\xe5\xbf\x97\xe7\x82\xb9"), "markers.json", "Marker Files (*.json *.txt)");
                    if (path.isEmpty()) return;
                    std::string spath = path.toStdString();
                    std::vector<osg::Vec3> markers;  // TODO: 从扫描数据获取
                    if (file_io::exportMarkers(spath, markers))
                        statusBar()->showMessage(QStringLiteral("\xe5\xaf\xbc\xe5\x87\xba\xe6\xa0\x87\xe5\xbf\x97\xe7\x82\xb9\xe6\x88\x90\xe5\x8a\x9f"));
                    else
                        statusBar()->showMessage(QStringLiteral("\xe5\xaf\xbc\xe5\x87\xba\xe5\xa4\xb1\xe8\xb4\xa5"));
                });
                menu.addAction(QStringLiteral("\xe5\xaf\xbc\xe5\x87\xba\xe7\x82\xb9\xe4\xba\x91"), [this]() {
                    QString path = QFileDialog::getSaveFileName(this, QStringLiteral("\xe5\xaf\xbc\xe5\x87\xba\xe7\x82\xb9\xe4\xba\x91"), "pointcloud.ply", "Point Cloud (*.ply *.pcd *.xyz)");
                    if (path.isEmpty()) return;
                    std::string spath = path.toStdString();
                    std::vector<osg::Vec3> points;  // TODO: 从扫描数据获取
                    if (file_io::exportPointCloud(spath, points))
                        statusBar()->showMessage(QStringLiteral("\xe5\xaf\xbc\xe5\x87\xba\xe7\x82\xb9\xe4\xba\x91\xe6\x88\x90\xe5\x8a\x9f"));
                    else
                        statusBar()->showMessage(QStringLiteral("\xe5\xaf\xbc\xe5\x87\xba\xe5\xa4\xb1\xe8\xb4\xa5"));
                });
                menu.addAction(QStringLiteral("\xe5\xaf\xbc\xe5\x87\xba\xe7\xbd\x91\xe6\xa0\xbc"), [this]() {
                    QString path = QFileDialog::getSaveFileName(this, QStringLiteral("\xe5\xaf\xbc\xe5\x87\xba\xe7\xbd\x91\xe6\xa0\xbc"), "mesh.stl", "Mesh (*.stl *.obj)");
                    if (path.isEmpty()) return;
                    std::string spath = path.toStdString();
                    file_io::MeshData mesh;  // TODO: 从后处理结果获取
                    if (file_io::exportMesh(spath, mesh))
                        statusBar()->showMessage(QStringLiteral("\xe5\xaf\xbc\xe5\x87\xba\xe7\xbd\x91\xe6\xa0\xbc\xe6\x88\x90\xe5\x8a\x9f"));
                    else
                        statusBar()->showMessage(QStringLiteral("\xe5\xaf\xbc\xe5\x87\xba\xe5\xa4\xb1\xe8\xb4\xa5"));
                });
                menu.addAction(QStringLiteral("\xe5\xaf\xbc\xe5\x87\xba\xe5\xb7\xa5\xe7\xa8\x8b\xe6\x96\x87\xe4\xbb\xb6"), [this]() {
                    QString path = QFileDialog::getSaveFileName(this, QStringLiteral("\xe5\xaf\xbc\xe5\x87\xba\xe5\xb7\xa5\xe7\xa8\x8b\xe6\x96\x87\xe4\xbb\xb6"), "project.leadscan", "Project (*.leadscan)");
                    if (!path.isEmpty()) statusBar()->showMessage(QStringLiteral("\xe5\xaf\xbc\xe5\x87\xba: ") + path);
                });

                QPoint pos = btn->mapToGlobal(QPoint(btn->width() + 4, 0));
                menu.exec(pos);
            });
        }

        // 标点扫描 / 面片扫描 / 点云扫描：toggle 切换
        if (i == 2) {
            connect(btn, &QPushButton::clicked, this, [this, btn]() {
                if (m_scanModeIdx == 2) { stopScan(); }
                else { onScanMarkers(); }
            });
        } else if (i == 3) {
            connect(btn, &QPushButton::clicked, this, [this, btn]() {
                if (m_scanModeIdx == 3) { stopScan(); }
                else { onScanMesh(); }
            });
        } else if (i == 4) {
            connect(btn, &QPushButton::clicked, this, [this, btn]() {
                if (m_scanModeIdx == 4) { stopScan(); }
                else { onScanPointCloud(); }
            });
        } else if (i == 5) {
            connect(btn, &QPushButton::clicked, this, [this]() {
                if (m_scanModeIdx == 5) { stopScan(); }
                else { onScanMesh(); m_scanModeIdx = 5; }
            });
        }

        if (i == 0) {
            layout->addSpacing(16);
            QFrame *separator = new QFrame();
            separator->setFixedWidth(1);
            separator->setFixedHeight(36);
            separator->setStyleSheet("background-color: #C0C0C0; border: none;");
            layout->addWidget(separator);
            layout->addSpacing(16);
        }
        if (i == 4) {
            QFrame *separator = new QFrame();
            separator->setFixedWidth(1);
            separator->setFixedHeight(36);
            separator->setStyleSheet("background-color: #C0C0C0; border: none;");
            layout->addWidget(separator);
        }
    }

    layout->addStretch();
    return bar;
}

QWidget *MainWindow::createLeftPanel()
{
    QWidget *innerPanel = new QWidget();
    innerPanel->setObjectName("leftPanelInner");
    innerPanel->setMinimumWidth(160);
    QVBoxLayout *innerLayout = new QVBoxLayout(innerPanel);
    innerLayout->setContentsMargins(0, 0, 0, 0);
    innerLayout->setSpacing(0);

    innerLayout->addWidget(createProjectSection(), 0);
    innerLayout->addWidget(createParamSection(), 1);
    innerLayout->addWidget(createInfoSection(), 0);

    QScrollArea *scrollArea = new QScrollArea();
    scrollArea->setObjectName("leftPanelScroll");
    scrollArea->setWidget(innerPanel);
    scrollArea->setWidgetResizable(true);
    scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    scrollArea->setMinimumWidth(160);
    scrollArea->setStyleSheet(
        "QScrollArea#leftPanelScroll { border: none; background: transparent; }"
        "QScrollBar:vertical { width: 4px; background: transparent; }"
        "QScrollBar::handle:vertical { background: #888888; border-radius: 2px; min-height: 20px; }"
        "QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0px; }"
        "QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical { background: none; }"
    );
    return scrollArea;
}

QWidget *MainWindow::createProjectSection()
{
    QWidget *section = new QWidget();
    section->setObjectName("projectSection");
    QVBoxLayout *layout = new QVBoxLayout(section);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);

    QWidget *header = new QWidget();
    header->setFixedHeight(28);
    header->setStyleSheet("background-color: #E1E1E1; border-bottom: 1px solid #C0C0C0; margin: 0px; padding: 0px;");
    QHBoxLayout *headerLayout = new QHBoxLayout(header);
    headerLayout->setContentsMargins(8, 0, 8, 0);

    QLabel *iconLbl = new QLabel();
    iconLbl->setPixmap(renderSvg(":/icons/resources/icons/项目管理-黑-14.svg", 11));
    headerLayout->addWidget(iconLbl);

    QLabel *title = new QLabel(QStringLiteral("项目树形结构"));
    title->setObjectName("sectionTitle");
    headerLayout->addWidget(title);
    headerLayout->addStretch();
    layout->addWidget(header);

    m_projectTree = new QTreeWidget();
    m_projectTree->setObjectName("projectTree");
    m_projectTree->setHeaderHidden(false);
    m_projectTree->setHeaderLabel(QStringLiteral("当前工程"));
    QFont headerFont = m_projectTree->header()->font();
    headerFont.setBold(true);
    m_projectTree->header()->setFont(headerFont);
    m_projectTree->setMinimumHeight(60);
    m_projectTree->setIndentation(16);
    m_projectTree->setStyleSheet("QTreeWidget { border: none; margin: 0px; padding: 0px; } QTreeWidget::item { padding: 0px; margin: 0px; }");

    QTreeWidgetItem *markerRoot = new QTreeWidgetItem(QStringList() << QStringLiteral("标记点列表 (124)"));
    markerRoot->setIcon(0, QIcon(renderSvg(":/icons/resources/icons/icon/left/marklist-black-11.svg", 14)));
    QTreeWidgetItem *m1 = new QTreeWidgetItem(markerRoot, QStringList() << QStringLiteral("标记点 001"));
    m1->setIcon(0, QIcon(renderSvg(":/icons/resources/icons/icon/left/marklist-black-11.svg", 11)));
    QTreeWidgetItem *m2 = new QTreeWidgetItem(markerRoot, QStringList() << QStringLiteral("标记点 002"));
    m2->setIcon(0, QIcon(renderSvg(":/icons/resources/icons/icon/left/marklist-black-11.svg", 11)));
    m_projectTree->addTopLevelItem(markerRoot);

    QTreeWidgetItem *cloudRoot = new QTreeWidgetItem(QStringList() << QStringLiteral("点云/三角面列表"));
    cloudRoot->setIcon(0, QIcon(renderSvg(":/icons/resources/icons/icon/left/cloudlist-black-11.svg", 14)));
    m_cloudItem001 = new QTreeWidgetItem(cloudRoot, QStringList() << QStringLiteral("点云数据 001"));
    m_cloudItem001->setIcon(0, QIcon(renderSvg(":/icons/resources/icons/icon/left/cloudlist-black-11.svg", 11)));
    QTreeWidgetItem *c2 = new QTreeWidgetItem(cloudRoot, QStringList() << QStringLiteral("三角面 001"));
    c2->setIcon(0, QIcon(renderSvg(":/icons/resources/icons/icon/left/cloudlist-black-11.svg", 11)));
    m_projectTree->addTopLevelItem(cloudRoot);

    QTreeWidgetItem* lineRoot = new QTreeWidgetItem(QStringList() << QStringLiteral("特征线列表"));
    lineRoot->setIcon(0, QIcon(renderSvg(":/icons/resources/icons/icon/left/marklist-black-11.svg", 14)));
    QTreeWidgetItem *l1 = new QTreeWidgetItem(lineRoot, QStringList() << QStringLiteral("特征线 001"));
    l1->setIcon(0, QIcon(renderSvg(":/icons/resources/icons/icon/left/marklist-black-11.svg", 11)));
    QTreeWidgetItem *l2 = new QTreeWidgetItem(lineRoot, QStringList() << QStringLiteral("特征线 002"));
    l2->setIcon(0, QIcon(renderSvg(":/icons/resources/icons/icon/left/marklist-black-11.svg", 11)));
    m_projectTree->addTopLevelItem(lineRoot);

    layout->addWidget(m_projectTree);
    return section;
}

QWidget *MainWindow::createParamSection()
{
    QWidget *section = new QWidget();
    section->setObjectName("paramSection");
    QVBoxLayout *layout = new QVBoxLayout(section);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);

    QWidget *header = new QWidget();
    header->setFixedHeight(28);
    header->setStyleSheet("background-color: #E1E1E1; border-bottom: 1px solid #C0C0C0;");
    QHBoxLayout *headerLayout = new QHBoxLayout(header);
    headerLayout->setContentsMargins(8, 0, 8, 0);
    QLabel *iconLbl = new QLabel();
    iconLbl->setPixmap(renderSvg(":/icons/resources/icons/icon/left/parapanel-black-11.svg", 11));
    headerLayout->addWidget(iconLbl);
    QLabel *title = new QLabel(QStringLiteral("参数面板"));
    title->setObjectName("sectionTitle");
    headerLayout->addWidget(title);
    headerLayout->addStretch();
    layout->addWidget(header);

    QWidget *tabBar = new QWidget();
    tabBar->setFixedHeight(28);
    QHBoxLayout *tabLayout = new QHBoxLayout(tabBar);
    tabLayout->setContentsMargins(0, 0, 0, 0);
    tabLayout->setSpacing(0);
    QStringList tabs = {QStringLiteral("自由"), QStringLiteral("推荐"), QStringLiteral("自定义")};
    for (int i = 0; i < tabs.size(); ++i) {
        QPushButton *tab = new QPushButton(tabs[i]);
        tab->setObjectName(i == 0 ? "paramTabActive" : "paramTab");
        tab->setFixedHeight(28);
        tab->setStyleSheet("background-color: #FFFFFF; color: #000000; border: none; border-radius: 0px;");
        tabLayout->addWidget(tab);
    }
    layout->addWidget(tabBar);

    QWidget *slidersWidget = new QWidget();
    slidersWidget->setStyleSheet("background-color: #FFFFFF;");
    slidersWidget->setMinimumHeight(200);
    QVBoxLayout *slidersLayout = new QVBoxLayout(slidersWidget);
    slidersLayout->setContentsMargins(8, 4, 8, 4);
    slidersLayout->setSpacing(0);

    struct SliderItem { QString name; int val; int min; int max; };
    QList<SliderItem> sliders = {
        {QStringLiteral("参数1：曝光/亮度"), 65, 0, 100},
        {QStringLiteral("参数2：点云分辨率"), 50, 0, 100},
        {QStringLiteral("参数3：滤波强度"), 30, 0, 100},
        {QStringLiteral("参数4：拼接平滑度"), 50, 0, 100},
        {QStringLiteral("参数5：纹理映射"), 20, 0, 100},
        {QStringLiteral("参数6：细节保留"), 10, 0, 100}
    };

    for (const auto &s : sliders) {
        QWidget *row = new QWidget();
        row->setMinimumHeight(40);
        QVBoxLayout *rowLayout = new QVBoxLayout(row);
        rowLayout->setContentsMargins(0, 0, 0, 0);
        rowLayout->setSpacing(0);

        QLabel *label = new QLabel(s.name);
        label->setObjectName("paramLabel");
        label->setMinimumHeight(12);
        label->setContentsMargins(0, 0, 0, 0);
        rowLayout->addWidget(label);

        QHBoxLayout *controlLayout = new QHBoxLayout();
        controlLayout->setSpacing(6);
        QSlider *slider = new QSlider(Qt::Horizontal);
        slider->setObjectName("paramSlider");
        slider->setRange(s.min, s.max);
        slider->setValue(s.val);
        slider->setStyleSheet(
            "QSlider::groove:horizontal { height: 4px; background: #E1E1E1; border-radius: 2px; }"
            "QSlider::handle:horizontal { background: #900021; width: 12px; height: 12px; margin: -5px 0px; border-radius: 6px; border: none; }"
        );
        controlLayout->addWidget(slider, 1);

        QLabel *valueLbl = new QLabel(QString::number(s.val));
        valueLbl->setObjectName("paramValue");
        valueLbl->setFixedWidth(48);
        valueLbl->setFixedHeight(20);
        valueLbl->setAlignment(Qt::AlignCenter);
        valueLbl->setStyleSheet("border: 1px solid #C0C0C0; border-radius: 4px; background-color: #FFFFFF; color: #000000;");
        QObject::connect(slider, &QSlider::valueChanged, valueLbl, [valueLbl](int val) {
            valueLbl->setText(QString::number(val));
        });
        controlLayout->addWidget(valueLbl);

        rowLayout->addLayout(controlLayout);
        slidersLayout->addWidget(row);
    }

    layout->addWidget(slidersWidget, 1);
    return section;
}

QWidget *MainWindow::createInfoSection()
{
    QWidget *section = new QWidget();
    section->setObjectName("infoSection");
    section->setStyleSheet("background-color: #FFFFFF;");
    QVBoxLayout *layout = new QVBoxLayout(section);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);

    QWidget *header = new QWidget();
    header->setFixedHeight(28);
    header->setStyleSheet("background-color: #E1E1E1; border-bottom: 1px solid #C0C0C0;");
    QHBoxLayout *headerLayout = new QHBoxLayout(header);
    headerLayout->setContentsMargins(8, 0, 8, 0);
    QLabel *iconLbl = new QLabel();
    iconLbl->setPixmap(renderSvg(":/icons/resources/icons/icon/left/infopanel-black-11.svg", 11));
    headerLayout->addWidget(iconLbl);
    QLabel *title = new QLabel(QStringLiteral("系统信息"));
    title->setObjectName("sectionTitle");
    headerLayout->addWidget(title);
    headerLayout->addStretch();
    layout->addWidget(header);

    QGridLayout *gridLayout = new QGridLayout();
    gridLayout->setContentsMargins(6, 6, 6, 6);
    gridLayout->setSpacing(6);

    struct InfoItem { QString icon; QString label; };
    QList<InfoItem> infos = {
        {"WIFIstate-black-11",   QStringLiteral("连接状态")},
        {"cloudnumber-black-11", QStringLiteral("点云数量")},
        {"infopanel-black-11",   QStringLiteral("帧率 (FPS)")},
        {"temprature-black-11",  QStringLiteral("设备温度")},
        {"cloudlist-black-11",   QStringLiteral("CPU占用率")},
        {"memory-black-11",      QStringLiteral("内存状态")}
    };

    QLabel** labelPtrs[] = {
        &m_infoConnLabel, &m_infoPointCloudLabel, &m_infoFpsLabel,
        &m_infoTempLabel, &m_infoCpuLabel, &m_infoMemLabel
    };

    for (int i = 0; i < infos.size(); ++i) {
        QWidget *card = new QWidget();
        card->setObjectName("infoCard");
        card->setStyleSheet("QWidget#infoCard { background-color: #FFFFFF; border: 1px solid #C0C0C0; border-radius: 10px; }");
        QVBoxLayout *cardLayout = new QVBoxLayout(card);
        cardLayout->setContentsMargins(9, 12, 9, 12);
        cardLayout->setSpacing(4);

        QHBoxLayout *labelRow = new QHBoxLayout();
        QLabel *icon = new QLabel();
        icon->setPixmap(renderSvg(QString(":/icons/resources/icons/icon/left/%1.svg").arg(infos[i].icon), 11));
        labelRow->addWidget(icon);
        QLabel *lbl = new QLabel(infos[i].label);
        lbl->setObjectName("infoCardLabel");
        labelRow->addWidget(lbl);
        labelRow->addStretch();
        cardLayout->addLayout(labelRow);

        QLabel *val = new QLabel("--");
        val->setObjectName("infoCardValue");
        val->setFixedHeight(30);
        QFont valFont = val->font();
        valFont.setBold(true);
        val->setFont(valFont);
        cardLayout->addWidget(val);

        *labelPtrs[i] = val;
        gridLayout->addWidget(card, i / 2, i % 2);
    }

    layout->addLayout(gridLayout, 1);
    return section;
}

QWidget *MainWindow::create3DViewArea()
{
    QWidget *area = new QWidget();
    area->setObjectName("view3DArea");
    QVBoxLayout *layout = new QVBoxLayout(area);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);

    QWidget *viewContainer = new QWidget();
    QHBoxLayout *viewLayout = new QHBoxLayout(viewContainer);
    viewLayout->setContentsMargins(0, 0, 0, 0);
    viewLayout->setSpacing(0);

    QVBoxLayout *gradientLayout = new QVBoxLayout();
    gradientLayout->setContentsMargins(6, 10, 6, 10);

    QLabel *labelFar = new QLabel(QStringLiteral("远"));
    labelFar->setAlignment(Qt::AlignCenter);
    labelFar->setStyleSheet("color: #0066FF; font-size: 14px; font-weight: bold;");
    gradientLayout->addWidget(labelFar);

    ArrowSlider *rangeSlider = new ArrowSlider(Qt::Vertical);
    rangeSlider->setObjectName("rangeSlider");
    rangeSlider->setRange(0, 100);
    rangeSlider->setValue(50);
    rangeSlider->setFixedWidth(48);
    rangeSlider->setMinimumHeight(200);
    rangeSlider->setGroovePixmap(renderSvg(":/icons/resources/icons/div.color-gradient-bar.svg", 24, 800));
    rangeSlider->setStyleSheet(
        "QSlider#rangeSlider::groove:vertical { background: transparent; }"
        "QSlider#rangeSlider::handle:vertical { background: transparent; width: 20px; height: 16px; }"
        "QSlider#rangeSlider::sub-page:vertical { background: transparent; }"
        "QSlider#rangeSlider::add-page:vertical { background: transparent; }"
    );
    gradientLayout->addWidget(rangeSlider, 1);

    QLabel *labelNear = new QLabel(QStringLiteral("近"));
    labelNear->setAlignment(Qt::AlignCenter);
    labelNear->setStyleSheet("color: #FF0000; font-size: 14px; font-weight: bold;");
    gradientLayout->addWidget(labelNear);

    viewLayout->addLayout(gradientLayout);

    m_3dView = new OSGWidget();
    connect(m_3dView, &OSGWidget::streamProgress, this,
        [this](int loaded, int total)
        {
            if (m_cloudItem001)
                m_cloudItem001->setText(0, QStringLiteral("点云数据 001 (%1)").arg(loaded));
        });
    viewLayout->addWidget(m_3dView, 1);

    layout->addWidget(viewContainer, 1);

    m_3dViewArea = area;
    return area;
}

QWidget *MainWindow::createBottomToolBar()
{
    QWidget *bar = new QWidget();
    bar->setObjectName("bottomToolBar");
    bar->setFixedHeight(47);
    QHBoxLayout *layout = new QHBoxLayout(bar);
    layout->setContentsMargins(10, 0, 10, 0);
    layout->setSpacing(0);
    layout->addStretch();

    QWidget *container = new QWidget();
    container->setObjectName("selectionToolBar");
    container->setStyleSheet("border: none;");
    QHBoxLayout *containerLayout = new QHBoxLayout(container);
    containerLayout->setContentsMargins(0, 0, 0, 0);
    containerLayout->setSpacing(2);
    containerLayout->setAlignment(Qt::AlignVCenter);

    struct SelBtn { QString iconBlack; };
    QList<SelBtn> selButtons = {
        {"allselect (3)"}, {"antiselect (1)"}, {"cancelselect (3)"},
        {"cloudandtriangleselect (1)"}, {"markandtriangleselect"},
        {"lasso (3)"}, {"markselect (1)"},
        {"surfaceselect (1)"},{"throwselect (1)"},
        {"delete (3)"},
    };

    for (int i = 0; i < selButtons.size(); ++i) {
        QPushButton *btn = new QPushButton();
        btn->setObjectName("selectionButton");
        btn->setFixedSize(40, 40);
        btn->setStyleSheet("border: none; margin: 0px; padding: 0px;");
        
        QPixmap pix = renderSvg(QString(":/icons/resources/icons/icon/bottom/%1.svg").arg(selButtons[i].iconBlack), 28);
        
        btn->setIcon(QIcon(pix));
        btn->setIconSize(QSize(28, 28));
        btn->setContentsMargins(0, 0, 0, 0);
        btn->setStyleSheet("border: none; margin: 0px; padding: 0px;");
        if (i == 0) btn->setProperty("active", true);
        containerLayout->addWidget(btn);
        m_selectionButtons.append(btn);

        if (i == 1 || i == 4 || i == 8) {
            QFrame *separator = new QFrame();
            separator->setFixedWidth(1);
            separator->setFixedHeight(28);
            separator->setStyleSheet("background-color: #C0C0C0; border: none;");
            containerLayout->addWidget(separator);
        }
    }

    layout->addWidget(container);
    layout->addStretch();

    // Wire up the delete button (index 9) — lasso-to-delete mode
    if (m_selectionButtons.size() > 9)
    {
        connect(m_selectionButtons[9], &QPushButton::clicked, this, [this]()
        {
            m_3dView->enterLassoDeleteMode();
        });
    }

    // Ctrl+Z undo
    QShortcut* undoShortcut = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_Z), this);
    connect(undoShortcut, &QShortcut::activated, this, [this]()
    {
        m_3dView->undoDelete();
    });

    return bar;
}

QPushButton *MainWindow::createNavButton(const QString &text, const QString &)
{
    QPushButton *btn = new QPushButton(text);
    btn->setObjectName("navButton");
    btn->setFixedHeight(38);
    return btn;
}

QPushButton *MainWindow::createToolButton(const QString &iconBlack, const QString &iconRed,
                                            const QString &iconGray, const QString &text)
{
    QPushButton *btn = new QPushButton();
    btn->setObjectName("toolButton");
    btn->setFixedSize(64, 56);
    QVBoxLayout *btnLayout = new QVBoxLayout(btn);
    btnLayout->setContentsMargins(0, 4, 0, 4);
    btnLayout->setSpacing(2);
    btnLayout->setAlignment(Qt::AlignCenter);

    QLabel *iconLbl = new QLabel();
    iconLbl->setPixmap(renderSvg(QString(":/icons/resources/icons/%1.svg").arg(iconBlack), 20));
    iconLbl->setAlignment(Qt::AlignCenter);
    btnLayout->addWidget(iconLbl);

    QLabel *textLbl = new QLabel(text);
    textLbl->setObjectName("toolButtonText");
    textLbl->setAlignment(Qt::AlignCenter);
    textLbl->setFixedWidth(60);
    btnLayout->addWidget(textLbl);

    return btn;
}

QPushButton *MainWindow::createSelectionButton(const QString &iconFile1, const QString &iconFile2,
                                                 const QString &iconFile3)
{
    QPushButton *btn = new QPushButton();
    btn->setObjectName("selectionButton");
    btn->setFixedSize(44, 32);
    btn->setIcon(QIcon(renderSvg(iconFile1, 14)));
    btn->setIconSize(QSize(14, 14));
    return btn;
}

void MainWindow::setButtonGroupExclusive(QList<QPushButton*> buttons)
{
    for (QPushButton *btn : buttons) {
        connect(btn, &QPushButton::clicked, this, [this, btn, buttons]() {
            setActiveButton(btn, buttons);
        });
    }
}

void MainWindow::setActiveButton(QPushButton *btn, QList<QPushButton*> group)
{
    for (QPushButton *b : group) {
        b->setProperty("active", false);
        b->style()->unpolish(b);
        b->style()->polish(b);
    }
    btn->setProperty("active", true);
    btn->style()->unpolish(btn);
    btn->style()->polish(btn);
}

bool MainWindow::eventFilter(QObject *obj, QEvent *event)
{
    if (obj->objectName() == "floatingToolbar") {
        if (event->type() == QEvent::MouseButtonPress) {
            QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
            if (mouseEvent->button() == Qt::LeftButton) {
                // 获取当前窗口的位置
                QPoint globalPos = mouseEvent->globalPos();
                // 获取窗口的位置
                QWidget *widget = qobject_cast<QWidget*>(obj);
                if (widget) {
                    QPoint windowPos = widget->window()->pos();
                    // 计算鼠标在窗口内的相对位置
                    QPoint relativePos = globalPos - windowPos;
                    
                    // 保存拖动状态和位置
                    bool dragging = true;
                    QPoint dragPosition = relativePos;
                    
                    // 设置拖动标志和位置
                    obj->setProperty("dragging", QVariant(dragging));
                    obj->setProperty("dragPosition", QVariant(dragPosition));
                    
                    return true;
                }
            }
        } else if (event->type() == QEvent::MouseMove) {
            QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
            if (mouseEvent->buttons() & Qt::LeftButton) {
                bool dragging = obj->property("dragging").toBool();
                QPoint dragPosition = obj->property("dragPosition").toPoint();
                
                if (dragging) {
                    // 计算新的窗口位置
                    QPoint globalPos = mouseEvent->globalPos();
                    QPoint newPos = globalPos - dragPosition;
                    
                    // 移动窗口
                    QWidget *widget = qobject_cast<QWidget*>(obj);
                    if (widget) {
                        widget->window()->move(newPos);
                    }
                    
                    return true;
                }
            }
        } else if (event->type() == QEvent::MouseButtonRelease) {
            QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
            if (mouseEvent->button() == Qt::LeftButton) {
                obj->setProperty("dragging", QVariant());
                obj->setProperty("dragPosition", QVariant());
                
                return true;
            }
        }
    }
    
    return QMainWindow::eventFilter(obj, event);
}

// ============================================================================
// 系统信息面板 — 定时采集真实数据
// ============================================================================
void MainWindow::startInfoTimer()
{
    // 初始化 CPU 时间戳
    FILETIME idleTime, kernelTime, userTime;
    if (GetSystemTimes(&idleTime, &kernelTime, &userTime)) {
        auto toU64 = [](const FILETIME& ft) -> uint64_t {
            ULARGE_INTEGER u;
            u.LowPart = ft.dwLowDateTime;
            u.HighPart = ft.dwHighDateTime;
            return u.QuadPart;
        };
        m_prevCpuIdle = static_cast<double>(toU64(idleTime));
        m_prevCpuKernel = static_cast<double>(toU64(kernelTime));
        m_prevCpuUser = static_cast<double>(toU64(userTime));
    }

    m_infoTimer = new QTimer(this);
    connect(m_infoTimer, &QTimer::timeout, this, &MainWindow::updateInfoSection);
    m_infoTimer->start(1000);

    // 3D 视图定时刷新（从 PointCloudBuffer 直读快照）
    QTimer* cloudTimer = new QTimer(this);
    connect(cloudTimer, &QTimer::timeout, this, [this]() {
        if (!m_appCtx || !m_3dView) return;
        auto* pcb = m_appCtx->pointCloudBuffer();
        if (!pcb || pcb->getTotalPointCount() == 0) return;
        static int lastCount = 0;
        int curCount = pcb->getTotalPointCount();
        if (curCount != lastCount) {
            m_3dView->loadFromPointCloudBuffer(pcb);
            lastCount = curCount;
            if (m_cloudItem001)
                m_cloudItem001->setText(0, QStringLiteral("点云数据 001 (%1)").arg(curCount));
        }
    });
    cloudTimer->start(500);
}

void MainWindow::updateInfoSection()
{
    static int updateCount = 0;
    ++updateCount;

    // === 先更新 CPU 和内存（纯 Windows API，不依赖任何框架组件）===
    if (m_infoCpuLabel) {
        FILETIME idleTime, kernelTime, userTime;
        if (GetSystemTimes(&idleTime, &kernelTime, &userTime)) {
            auto toU64 = [](const FILETIME& ft) -> uint64_t {
                ULARGE_INTEGER u; u.LowPart = ft.dwLowDateTime; u.HighPart = ft.dwHighDateTime;
                return u.QuadPart;
            };
            double idle = static_cast<double>(toU64(idleTime));
            double kernel = static_cast<double>(toU64(kernelTime));
            double user = static_cast<double>(toU64(userTime));
            double idleDiff = idle - m_prevCpuIdle;
            double total = (kernel - m_prevCpuKernel) + (user - m_prevCpuUser);
            m_prevCpuIdle = idle; m_prevCpuKernel = kernel; m_prevCpuUser = user;
            double usage = (total > 0) ? ((total - idleDiff) / total * 100.0) : 0.0;
            if (usage < 0) usage = 0; if (usage > 100) usage = 100;
            m_infoCpuLabel->setText(QString::number(usage, 'f', 1) + " %");
        }
    }

    if (m_infoMemLabel) {
        MEMORYSTATUSEX mem;
        mem.dwLength = sizeof(mem);
        if (GlobalMemoryStatusEx(&mem)) {
            double totalGB = static_cast<double>(mem.ullTotalPhys) / (1024.0 * 1024.0 * 1024.0);
            double usedGB = totalGB - static_cast<double>(mem.ullAvailPhys) / (1024.0 * 1024.0 * 1024.0);
            m_infoMemLabel->setText(QString("%1 / %2 GB (#%3)")
                .arg(usedGB, 0, 'f', 1).arg(totalGB, 0, 'f', 1).arg(updateCount));
        }
    }

    // === 再更新设备相关状态 ===
    Scanner::data::DeviceStateCache* dsc = nullptr;
    Scanner::data::PointCloudBuffer* pcb = nullptr;
    if (m_appCtx) {
        dsc = m_appCtx->deviceStateCache();
        pcb = m_appCtx->pointCloudBuffer();
    }

    // 1. 连接状态 — 从 DeviceStateCache 读
    if (m_infoConnLabel) {
        bool camConnected = false;
        if (dsc) {
            auto camState = dsc->getState("Camera");
            camConnected = (camState.state == Scanner::DeviceState::Connected ||
                            camState.state == Scanner::DeviceState::Streaming);
        }
        m_infoConnLabel->setText(camConnected ? "已连接" : "未连接");
        m_infoConnLabel->setStyleSheet(camConnected ? "color: #00AA00;" : "color: #CC0000;");
    }

    // 2. 点云数量 — 从 PointCloudBuffer 读
    if (m_infoPointCloudLabel) {
        int count = pcb ? pcb->getTotalPointCount() : 0;
        m_infoPointCloudLabel->setText(QString::number(count));
    }

    // 3. 帧率 — 从 DeviceStateCache 或 FrameBuffer 水位
    if (m_infoFpsLabel) {
        double fps = dsc ? dsc->getFps("Camera") : 0.0;
        if (fps > 0.0) {
            m_infoFpsLabel->setText(QString::number(static_cast<int>(fps)) + " fps");
        } else if (m_appCtx && m_appCtx->frameBuffer()) {
            // 回退: 用 FrameBuffer 水位推算
            int level = m_appCtx->frameBuffer()->getBufferLevel();
            m_infoFpsLabel->setText(level > 0 ? QString::number(level) + " f" : "-- fps");
        } else {
            m_infoFpsLabel->setText("-- fps");
        }
    }

    // 4. 设备温度 — 从 DeviceStateCache 读（HardwareMonitor 写入）
    if (m_infoTempLabel) {
        double camTemp = dsc ? dsc->getTemperature("Camera") : 0.0;
        double mcuTemp = dsc ? dsc->getTemperature("MCU") : 0.0;
        double temp = std::max(camTemp, mcuTemp);
        if (temp > 0.0) {
            m_infoTempLabel->setText(QString::number(temp, 'f', 1) + " ℃");
            m_infoTempLabel->setStyleSheet(temp > 50.0 ? "color: #DDAA00;" : "");
        } else {
            m_infoTempLabel->setText("-- ℃");
        }
    }

    // 5. CPU 占用率 — Windows API
    if (m_infoCpuLabel) {
        FILETIME idleTime, kernelTime, userTime;
        if (GetSystemTimes(&idleTime, &kernelTime, &userTime)) {
            auto toU64 = [](const FILETIME& ft) -> uint64_t {
                ULARGE_INTEGER u;
                u.LowPart = ft.dwLowDateTime;
                u.HighPart = ft.dwHighDateTime;
                return u.QuadPart;
            };
            double idle = static_cast<double>(toU64(idleTime));
            double kernel = static_cast<double>(toU64(kernelTime));
            double user = static_cast<double>(toU64(userTime));

            double idleDiff = idle - m_prevCpuIdle;
            double kernelDiff = kernel - m_prevCpuKernel;
            double userDiff = user - m_prevCpuUser;
            double total = kernelDiff + userDiff;

            m_prevCpuIdle = idle;
            m_prevCpuKernel = kernel;
            m_prevCpuUser = user;

            double usage = (total > 0) ? ((total - idleDiff) / total * 100.0) : 0.0;
            if (usage < 0) usage = 0;
            if (usage > 100) usage = 100;
            m_infoCpuLabel->setText(QString::number(usage, 'f', 1) + " %");
        } else {
            m_infoCpuLabel->setText("-- %");
        }
    }

    // 6. 内存状态
    if (m_infoMemLabel) {
        MEMORYSTATUSEX mem;
        mem.dwLength = sizeof(mem);
        if (GlobalMemoryStatusEx(&mem)) {
            double totalGB = static_cast<double>(mem.ullTotalPhys) / (1024.0 * 1024.0 * 1024.0);
            double usedGB = totalGB - static_cast<double>(mem.ullAvailPhys) / (1024.0 * 1024.0 * 1024.0);
            m_infoMemLabel->setText(QString("%1 / %2 GB (#%3)")
                .arg(usedGB, 0, 'f', 1)
                .arg(totalGB, 0, 'f', 1)
                .arg(updateCount));
        } else {
            m_infoMemLabel->setText("-- / -- GB");
        }
    }
}