#pragma once

#include <QMainWindow>
#include <QPushButton>
#include <QLabel>
#include <QSlider>
#include <QSpinBox>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QStackedWidget>
#include <QPaintEvent>
#include <QSvgRenderer>
#include <opencv2/core.hpp>

#include "OSGWidget.h"

class ArrowSlider : public QSlider
{
    Q_OBJECT
public:
    explicit ArrowSlider(Qt::Orientation orientation, QWidget *parent = nullptr);
    void setGroovePixmap(const QPixmap &pixmap);

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    QPixmap m_groovePixmap;
};

class CalibDialog;
class CameraControl;
class LEADSCANSeries;
class AppContext;
namespace calib_display { class CalibBoard2D; }

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(AppContext* appCtx = nullptr, QWidget *parent = nullptr);
    ~MainWindow();

    AppContext* appCtx() { return m_appCtx; }

private:
    QWidget *createTitleBar();
    QWidget *createNavBar();
    QWidget *createToolBar();
    QWidget *createLeftPanel();
    QWidget *createProjectSection();
    QWidget *createParamSection();
    QWidget *createInfoSection();
    QWidget *create3DViewArea();
    QWidget *createBottomToolBar();
    QWidget *createCoordOverlay();

    QPushButton *createIconButton(const QString &iconBlack, const QString &iconRed, const QString &iconGray,
                                   const QString &text = QString(), int iconSize = 14, bool vertical = false);
    QPushButton *createNavButton(const QString &text, const QString &iconWhite);
    QPushButton *createToolButton(const QString &iconBlack, const QString &iconRed, const QString &iconGray,
                                   const QString &text);
    QPushButton *createSelectionButton(const QString &iconFile1, const QString &iconFile2, const QString &iconFile3);

    static QPixmap renderSvg(const QString &svgPath, int size);
    static QPixmap renderSvg(const QString &svgPath, int w, int h);
    void setupUILayout();
    void repositionFloatingToolbar();
    void setButtonGroupExclusive(QList<QPushButton*> buttons);
    void setActiveButton(QPushButton *btn, QList<QPushButton*> group);
    void createFloatingToolbar();

private slots:
    void onIntegrateTestClicked();
    void onReloadPointCloud();
    void onCalibDeviceClicked();
    void onScanClicked();

protected:
    bool eventFilter(QObject *obj, QEvent *event) override;
    void closeEvent(QCloseEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

    QList<QPushButton*> m_navLeftButtons;
    QList<QPushButton*> m_navRightButtons;
    QList<QPushButton*> m_toolButtons;
    QList<QPushButton*> m_selectionButtons;

    OSGWidget *m_3dView;
    QWidget *m_3dViewArea;
    QLabel *m_projectName;
    QTreeWidget *m_projectTree;
    QTreeWidgetItem *m_cloudItem001;
    QWidget *m_floatingToolbar;

    AppContext *m_appCtx = nullptr;

    // 标定分屏
    QWidget* m_calibSplitWidget = nullptr;
    calib_display::CalibBoard2D* m_calibBoard2D = nullptr;

    QWidget *m_integrateTestDialog;
    CalibDialog *m_calibDialog;
    struct CameraCalibResultHolder {
        bool success = false;
        // 简化：只存关键矩阵，避免头文件依赖calib_workflow.h
        cv::Mat cameraMatrixL, distCoeffsL, cameraMatrixR, distCoeffsR;
        cv::Mat R, T, R1, R2, P1, P2, Q;
        double intrinsicRMS = 0;
        double stereoReprojError = 0;
        bool hasTempTables = false;
    } m_lastCameraCalib;
    LEADSCANSeries *m_series = nullptr;

    // 系统信息面板
    QTimer *m_infoTimer = nullptr;
    QLabel *m_infoConnLabel = nullptr;
    QLabel *m_infoPointCloudLabel = nullptr;
    QLabel *m_infoFpsLabel = nullptr;
    QLabel *m_infoTempLabel = nullptr;
    QLabel *m_infoCpuLabel = nullptr;
    QLabel *m_infoMemLabel = nullptr;
    double m_prevCpuIdle = 0;
    double m_prevCpuKernel = 0;
    double m_prevCpuUser = 0;

    void startInfoTimer();
    void updateInfoSection();
};
