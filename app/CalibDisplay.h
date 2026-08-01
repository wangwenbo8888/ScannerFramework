#pragma once

#include <osg/Group>
#include <string>

namespace calib_display {

// 构建标定显示场景（坐标轴复用 OSGWidget 现有 overlay，不含相机/渲染设置）
// 场景内容：
//   - 标定板组：放大 2.2 + 绕 Z 顺时针 90° + Y+40 平移（板平面 + 网格 + 标志点）
//   - 扫描仪组：缩放 0.6 + 绕 Z 顺时针 90° + Y-220 平移（绿色目标 + 红色当前 STL 模型）
//   - 偏差 HUD（前后/左右/远近/俯仰/偏航/翻滚 + 色阶条）
// stlPath: 扫描仪 STL 模型路径
osg::Group* buildCalibScene(const std::string& stlPath);
} // namespace calib_display
