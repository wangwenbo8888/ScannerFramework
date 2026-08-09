#pragma once

#include <osg/Group>
#include <string>
#include <QWidget>
#include <QImage>
#include <opencv2/core.hpp>

namespace calib_display {

osg::Group* buildCalibScene(const std::string& stlPath);

// 2D 标定板控件（Qt 绘制，无 3D 渲染）
class CalibBoard2D : public QWidget {
public:
    explicit CalibBoard2D(QWidget* parent = nullptr);
protected:
    void paintEvent(QPaintEvent*) override;
};

// 相机预览弹窗
class CameraPreviewDialog : public QWidget {
public:
    explicit CameraPreviewDialog(QWidget* parent = nullptr);
    void updateFrames(const cv::Mat& left, const cv::Mat& right, const QString& status);
protected:
    void paintEvent(QPaintEvent*) override;
private:
    QImage m_leftImg, m_rightImg;
    QString m_statusText;
};

} // namespace calib_display
