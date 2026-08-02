#pragma once

#include <osg/Group>
#include <string>
#include <QWidget>

namespace calib_display {

osg::Group* buildCalibScene(const std::string& stlPath);

// 2D 标定板控件（Qt 绘制，无 3D 渲染）
class CalibBoard2D : public QWidget {
public:
    explicit CalibBoard2D(QWidget* parent = nullptr);
protected:
    void paintEvent(QPaintEvent*) override;
};

// 姿态偏差彩条（标签+数值+5段绿→红，横/竖可选）
class PoseBar : public QWidget {
public:
    enum Orient { Horizontal, Vertical };
    explicit PoseBar(const QString& title, Orient o, QWidget* parent = nullptr);
    void setValue(float val, float maxRange = 30.0f);
protected:
    void paintEvent(QPaintEvent*) override;
private:
    QString m_title;
    float m_val = 0.0f;
    float m_maxRange = 30.0f;
    Orient m_orient;
};

} // namespace calib_display
