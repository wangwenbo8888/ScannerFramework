#include "MainWindow.h"
#include "AppContext.h"
#include "data/DeviceStateCache.h"
#include "data/PointCloudBuffer.h"
#include "CalibDialog.h"
#include "CalibDisplay.h"
#include "IntegrateTestDialog.h"
#include "ScannerWindow.h"
#include "stubs/LEADSCANSeries.h"
#include "stubs/CameraControl.h"
#include "stubs/camera_calib_workflow.h"
#include "stubs/laser_calib_workflow.h"
#include "stubs/scan_workflow.h"
#include <osg/Vec3>
#include <osg/Matrix>
#include <osgGA/TrackballManipulator>
#include <QPainter>
#include <QApplication>
#include <QHeaderView>
#include <QScreen>
#include <QResizeEvent>
#include <QTimer>
#include <QScrollArea>
#include <QShortcut>
#include <QMessageBox>
#include <QStatusBar>
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

    if (!m_groovePixmap.isNull()) {
        QPixmap scaled = m_groovePixmap.scaled(24, height() - 20, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
        int gx = (width() - scaled.width()) / 2;
        p.drawPixmap(gx, 10, scaled);
    }

    QStyleOptionSlider opt;
    initStyleOption(&opt);
    QRect handleRect = style()->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderHandle, this);

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

    startInfoTimer();
}

MainWindow::~MainWindow() {}

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
    {
        FILE* f = fopen("E:/workfold/20260509intergrate/calib_debug.log", "a");
        if (f) { fprintf(f, "[%s] onCalibDeviceClicked ENTER m_3dView=%p\n", __TIME__, (void*)m_3dView); fclose(f); }
    }
    if (!m_calibDialog) {
        m_calibDialog = new CalibDialog(this);
        connect(m_calibDialog, &CalibDialog::cameraCalibClicked, this, [this]() {
            statusBar()->showMessage(QStringLiteral("相机标定：正在打开相机..."));

            auto* series = static_cast<LEADSCANSeries*>(m_integrateTestDialog);
            if (!series) {
                if (!m_integrateTestDialog) {
                    m_integrateTestDialog = new LEADSCANSeries();
                }
                series = static_cast<LEADSCANSeries*>(m_integrateTestDialog);
            }
            auto* cam = series->getCameraControl();
            if (!cam || !cam->isScannerCameraOpen()) {
                QMessageBox::warning(this, QStringLiteral("相机标定"), QStringLiteral("请先在集成测试中打开扫描相机"));
                return;
            }

            const int numFrames = 15;
            calibration::CameraCalibInput input;
            input.imageWidth = 2048;
            input.imageHeight = 1536;

            for (int i = 0; i < numFrames; ++i) {
                statusBar()->showMessage(QStringLiteral("采集标定图像 %1/%2").arg(i + 1).arg(numFrames));
                QApplication::processEvents();

                cv::Mat left, right;
                cam->GetScannerImages(left, right, 10000);
                if (!left.empty() && !right.empty()) {
                    input.leftImages.push_back(left.clone());
                    input.rightImages.push_back(right.clone());
                }
            }

            statusBar()->showMessage(QStringLiteral("正在执行相机标定算法..."));
            QApplication::processEvents();

            calibration::CameraCalibWorkflow workflow;
            workflow.setProgressCallback([this](int pct, const std::string& step) {
                statusBar()->showMessage(QString::fromStdString(step) + " (" + QString::number(pct) + "%)");
                QApplication::processEvents();
            });

            auto result = workflow.run(input);
            if (result.success) {
                QMessageBox::information(this, QStringLiteral("相机标定"),
                    QStringLiteral("标定完成!\n左重投影误差: %1\n右重投影误差: %2\n立体重投影误差: %3")
                    .arg(result.reprojError, 0, 'f', 4));
            } else {
                QMessageBox::warning(this, QStringLiteral("相机标定"), QString::fromStdString(result.message));
            }
        });
        connect(m_calibDialog, &CalibDialog::laserCalibClicked, this, [this]() {
            statusBar()->showMessage(QStringLiteral("激光线标定：正在打开相机..."));

            auto* series = static_cast<LEADSCANSeries*>(m_integrateTestDialog);
            if (!series) {
                if (!m_integrateTestDialog) {
                    m_integrateTestDialog = new LEADSCANSeries();
                }
                series = static_cast<LEADSCANSeries*>(m_integrateTestDialog);
            }
            auto* cam = series->getCameraControl();
            if (!cam || !cam->isScannerCameraOpen()) {
                QMessageBox::warning(this, QStringLiteral("激光线标定"), QStringLiteral("请先在集成测试中打开扫描相机"));
                return;
            }

            statusBar()->showMessage(QStringLiteral("采集激光线图像..."));
            QApplication::processEvents();

            cv::Mat left, right;
            cam->GetScannerImages(left, right, 10000);
            if (left.empty() || right.empty()) {
                QMessageBox::warning(this, QStringLiteral("激光线标定"), QStringLiteral("图像采集失败"));
                return;
            }

            statusBar()->showMessage(QStringLiteral("正在执行激光线标定算法..."));
            QApplication::processEvents();

            calibration::LaserCalibWorkflow workflow;
            workflow.setProgressCallback([this](int pct, const std::string& step) {
                statusBar()->showMessage(QString::fromStdString(step) + " (" + QString::number(pct) + "%)");
                QApplication::processEvents();
            });

            calibration::LaserCalibInput input;
            input.leftImage = left;
            input.rightImage = right;

            auto result = workflow.run(input);
            if (result.success) {
                QMessageBox::information(this, QStringLiteral("激光线标定"),
                    QStringLiteral("标定完成!\n激光线数: %1\n端点数: %2")
                    .arg(result.lineCount).arg(result.totalEndpoints));
            } else {
                QMessageBox::warning(this, QStringLiteral("激光线标定"), QString::fromStdString(result.message));
            }
        });
    }
    // 分屏：左 3D 扫描仪 + 右 2D 标定板
    if (m_3dView && m_3dViewArea) {
        // 隐藏悬浮工具条
        if (m_floatingToolbar) {
            m_floatingToolbar->setVisible(false);
            m_floatingToolbar->hide();
            m_floatingToolbar->move(-10000, -10000);
        }

        // 加载扫描仪 STL 到 3D 视图
        std::string stlTarget = "E:/workfold/framework/build/JEAMMSCAN.stl";
        osg::ref_ptr<osg::Group> scene = calib_display::buildCalibScene(stlTarget);
        m_3dView->setSceneData(scene);
        m_3dView->setCenterOverlayVisible(false);
        m_3dView->setCameraManipulator(new osgGA::TrackballManipulator());
        m_3dView->home();

        // 创建分屏：左 OSGWidget(3D) + 右 CalibBoard2D(2D)
        if (!m_calibSplitWidget) {
            m_calibSplitWidget = new QWidget();
            m_calibBoard2D = new calib_display::CalibBoard2D();

            // 找到 3D 视图的父布局
            auto* oldParent = m_3dView->parentWidget();
            auto* oldLayout = oldParent ? oldParent->layout() : nullptr;

            auto* splitLayout = new QHBoxLayout(m_calibSplitWidget);
            splitLayout->setContentsMargins(0, 0, 0, 0);
            splitLayout->setSpacing(0);

            // 把 m_3dView 从原父控件移到分屏左侧
            m_3dView->setParent(m_calibSplitWidget);
            splitLayout->addWidget(m_3dView, 3);      // 左：3D 扫描仪
            splitLayout->addWidget(m_calibBoard2D, 2); // 右：2D 标定板

            // 替换原布局内容
            if (oldLayout) {
                // 清空原布局中的渐变条等
                while (oldLayout->count() > 0) {
                    auto* item = oldLayout->takeAt(0);
                    if (item->widget()) item->widget()->setParent(m_3dViewArea);
                }
                oldLayout->addWidget(m_calibSplitWidget);
            }
        }
        m_calibSplitWidget->show();
        m_calibBoard2D->update();

        statusBar()->showMessage(QStringLiteral("标定显示模式"));
    }
}

void MainWindow::onScanClicked()
{
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

void MainWindow::closeEvent(QCloseEvent *event)
{
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
    if (!m_floatingToolbar || !m_3dView) return;
    if (!m_floatingToolbar->isVisible()) return;
    if (!m_floatingToolbar->isVisible()) return;  // 已隐藏则不处理
    m_floatingToolbar->adjustSize();
    QRect viewGeo = m_3dView->geometry();
    QPoint viewBottomCenter = m_3dView->mapToGlobal(QPoint(viewGeo.width() / 2, viewGeo.height()));
    int tbX = viewBottomCenter.x() - m_floatingToolbar->width() / 2;
    int tbY = viewBottomCenter.y() - m_floatingToolbar->height() - 10;
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