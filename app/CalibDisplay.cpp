// CalibDisplay.cpp
// 标定显示场景构建：标定板 + 标志点 + STL 姿态模型 + 偏差 HUD
// 移植自 calib_display_demo.cpp（去掉 main/相机/坐标轴，坐标轴复用 OSGWidget 现有 overlay）
#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#endif
#include "CalibDisplay.h"

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Array>
#include <osg/PrimitiveSet>
#include <osg/StateSet>
#include <osg/Material>
#include <osg/LineWidth>
#include <osg/MatrixTransform>
#include <osg/Camera>
#include <osg/BoundingBox>
#include <osg/BoundingSphere>
#include <osg/CopyOp>
#include <osgText/Text>
#include <osgDB/ReadFile>
#include <osgDB/Registry>

#include <cmath>
#include <cstdio>
#include <vector>
#include <string>

namespace calib_display {

static const float kBoardW = 180.0f;
static const float kBoardH = 120.0f;
static const float kGridSpacing = 30.0f;
static const float kMarkerRadius = 3.0f;
static const int   kMarkerCols = 7;
static const int   kMarkerRows = 6;

// 标定板矩形平面
static osg::Geometry* createBoardPlane()
{
    float hw = kBoardW * 0.5f;
    float hh = kBoardH * 0.5f;

    osg::ref_ptr<osg::Vec3Array> v = new osg::Vec3Array;
    v->push_back(osg::Vec3(-hw, -hh, 0));
    v->push_back(osg::Vec3(hw, -hh, 0));
    v->push_back(osg::Vec3(hw, hh, 0));
    v->push_back(osg::Vec3(-hw, hh, 0));

    osg::ref_ptr<osg::Vec3Array> n = new osg::Vec3Array;
    n->push_back(osg::Vec3(0, 0, 1));

    osg::ref_ptr<osg::Vec4Array> c = new osg::Vec4Array;
    c->push_back(osg::Vec4(0.7f, 0.7f, 0.7f, 1.0f));

    osg::ref_ptr<osg::Geometry> g = new osg::Geometry;
    g->setUseDisplayList(false);
    g->setVertexArray(v);
    g->setNormalArray(n);
    g->setNormalBinding(osg::Geometry::BIND_OVERALL);
    g->setColorArray(c);
    g->setColorBinding(osg::Geometry::BIND_OVERALL);
    g->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS, 0, 4));
    return g.release();
}

// 网格线
static osg::Geometry* createGridLines()
{
    float hw = kBoardW * 0.5f;
    float hh = kBoardH * 0.5f;

    osg::ref_ptr<osg::Vec3Array> v = new osg::Vec3Array;
    for (float x = -hw; x <= hw + 0.01f; x += kGridSpacing) {
        v->push_back(osg::Vec3(x, -hh, 0.01f));
        v->push_back(osg::Vec3(x, hh, 0.01f));
    }
    for (float y = -hh; y <= hh + 0.01f; y += kGridSpacing) {
        v->push_back(osg::Vec3(-hw, y, 0.01f));
        v->push_back(osg::Vec3(hw, y, 0.01f));
    }

    osg::ref_ptr<osg::Vec4Array> c = new osg::Vec4Array;
    c->push_back(osg::Vec4(0.5f, 0.5f, 0.5f, 1.0f));

    osg::ref_ptr<osg::Geometry> g = new osg::Geometry;
    g->setUseDisplayList(false);
    g->setVertexArray(v);
    g->setColorArray(c);
    g->setColorBinding(osg::Geometry::BIND_OVERALL);
    g->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, v->size()));

    osg::ref_ptr<osg::LineWidth> lw = new osg::LineWidth(1.0f);
    g->getOrCreateStateSet()->setAttribute(lw);
    g->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    return g.release();
}

// 标志点（圆点 + 编号），绿=已扫 红=未扫
static osg::Group* createMarkers()
{
    osg::ref_ptr<osg::Group> grp = new osg::Group;

    int totalMarkers = kMarkerCols * kMarkerRows;
    std::vector<osg::Vec3> positions;
    float xMargin = 15.0f;
    float yMargin = 15.0f;
    float xStep = (kBoardW - 2 * xMargin) / (kMarkerCols - 1);
    float yStep = (kBoardH - 2 * yMargin) / (kMarkerRows - 1);
    for (int row = 0; row < kMarkerRows; ++row) {
        for (int col = 0; col < kMarkerCols; ++col) {
            float x = -kBoardW * 0.5f + xMargin + col * xStep;
            float y = -kBoardH * 0.5f + yMargin + row * yStep;
            positions.push_back(osg::Vec3(x, y, 0.1f));
        }
    }

    std::vector<bool> scanned(totalMarkers, false);
    for (int i = 0; i < 15 && i < totalMarkers; ++i) scanned[i] = true;

    const int segs = 32;
    for (size_t i = 0; i < positions.size(); ++i) {
        float cx = positions[i].x();
        float cy = positions[i].y();
        float cz = positions[i].z();

        osg::ref_ptr<osg::Vec3Array> v = new osg::Vec3Array;
        for (int s = 0; s < segs; ++s) {
            float a = (float)s / segs * 2.0f * 3.14159265f;
            v->push_back(osg::Vec3(cx + kMarkerRadius * cosf(a),
                                   cy + kMarkerRadius * sinf(a), cz));
        }
        osg::ref_ptr<osg::Vec4Array> c = new osg::Vec4Array;
        c->push_back(scanned[i] ? osg::Vec4(0, 1, 0, 1) : osg::Vec4(1, 0.3f, 0.3f, 1));

        osg::ref_ptr<osg::Geometry> circle = new osg::Geometry;
        circle->setUseDisplayList(false);
        circle->setVertexArray(v);
        circle->setColorArray(c);
        circle->setColorBinding(osg::Geometry::BIND_OVERALL);
        circle->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POLYGON, 0, segs));
        circle->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
        geode->addDrawable(circle);

        osg::ref_ptr<osgText::Text> label = new osgText::Text;
        label->setText(std::to_string(i));
        label->setPosition(osg::Vec3(cx, cy, cz + 0.1f));
        label->setCharacterSize(12.0f);
        label->setAlignment(osgText::Text::CENTER_CENTER);
        label->setColor(osg::Vec4(1, 1, 1, 1));
        label->setAxisAlignment(osgText::Text::SCREEN);
        geode->addDrawable(label);

        grp->addChild(geode);
    }
    return grp.release();
}

// 递归计算节点包围盒
static osg::BoundingBox computeBoundingBox(osg::Node* node)
{
    osg::BoundingBox bb;
    if (auto* geode = dynamic_cast<osg::Geode*>(node)) {
        for (unsigned i = 0; i < geode->getNumDrawables(); ++i)
            bb.expandBy(geode->getDrawable(i)->getBoundingBox());
    }
    if (auto* group = dynamic_cast<osg::Group*>(node)) {
        for (unsigned i = 0; i < group->getNumChildren(); ++i)
            bb.expandBy(computeBoundingBox(group->getChild(i)));
    }
    return bb;
}

// 递归给节点所有 Geometry 设置统一颜色（关光照下也能显示）
static void applyColorRecursive(osg::Node* node, const osg::Vec4& color)
{
    if (auto* geode = dynamic_cast<osg::Geode*>(node)) {
        osg::ref_ptr<osg::Vec4Array> c = new osg::Vec4Array;
        c->push_back(color);
        for (unsigned i = 0; i < geode->getNumDrawables(); ++i) {
            osg::Geometry* geom = geode->getDrawable(i)->asGeometry();
            if (geom) {
                geom->setColorArray(c);
                geom->setColorBinding(osg::Geometry::BIND_OVERALL);
            }
        }
    }
    if (auto* group = dynamic_cast<osg::Group*>(node)) {
        for (unsigned i = 0; i < group->getNumChildren(); ++i)
            applyColorRecursive(group->getChild(i), color);
    }
}

// 手动解析 binary STL（绕过 osgDB 插件加载问题）
static osg::Geode* loadStlManual(const std::string& path)
{
    FILE* f = fopen(path.c_str(), "rb");
    if (!f) return nullptr;

    char header[80];
    if (fread(header, 1, 80, f) != 80) { fclose(f); return nullptr; }

    unsigned int numTris = 0;
    if (fread(&numTris, 4, 1, f) != 1) { fclose(f); return nullptr; }

    osg::ref_ptr<osg::Vec3Array> verts = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec3Array> norms = new osg::Vec3Array;
    verts->reserve(numTris * 3);
    norms->reserve(numTris * 3);

    for (unsigned int i = 0; i < numTris; ++i) {
        float n[3], v[9];
        unsigned short attr;
        if (fread(n, 4, 3, f) != 3) break;
        if (fread(v, 4, 9, f) != 9) break;
        if (fread(&attr, 2, 1, f) != 1) break;

        verts->push_back(osg::Vec3(v[0], v[1], v[2]));
        verts->push_back(osg::Vec3(v[3], v[4], v[5]));
        verts->push_back(osg::Vec3(v[6], v[7], v[8]));
        for (int k = 0; k < 3; ++k) norms->push_back(osg::Vec3(n[0], n[1], n[2]));
    }
    fclose(f);

    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    geom->setUseDisplayList(false);
    geom->setVertexArray(verts);
    geom->setNormalArray(norms);
    geom->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES, 0, (int)verts->size()));

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable(geom);
    printf("  loadStlManual: %u 三角面, %zu 顶点\n", numTris, verts->size());
    return geode.release();
}

// STL 模型（使用 STL 原始坐标，不做居中/缩放/平移，颜色数组着色）
static osg::MatrixTransform* createPoseModel(const std::string& stlPath,
                                             float r, float g, float b)
{
    osg::ref_ptr<osg::Node> geo = loadStlManual(stlPath);
    {
        FILE* f = fopen("E:/workfold/20260509intergrate/calib_debug.log", "a");
        if (f) { fprintf(f, "createPoseModel stl=%s geo=%p\n", stlPath.c_str(), (void*)geo.get()); fclose(f); }
    }
    if (!geo) {
        printf("  STL 加载失败: %s\n", stlPath.c_str());
        return nullptr;
    }

    osg::BoundingBox bb = computeBoundingBox(geo);
    printf("  STL 包围盒: X[%.1f, %.1f] Y[%.1f, %.1f] Z[%.1f, %.1f]\n",
           bb.xMin(), bb.xMax(), bb.yMin(), bb.yMax(), bb.zMin(), bb.zMax());

    auto* xform = new osg::MatrixTransform;
    xform->setMatrix(osg::Matrix::identity());

    auto* cloned = dynamic_cast<osg::Node*>(geo->clone(osg::CopyOp::DEEP_COPY_NODES));
    osg::Node* child = cloned ? cloned : geo.get();
    applyColorRecursive(child, osg::Vec4(r, g, b, 0.6f));
    xform->addChild(child);

    osg::StateSet* ss = xform->getOrCreateStateSet();
    ss->setMode(GL_CULL_FACE, osg::StateAttribute::OFF);
    ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    ss->setMode(GL_BLEND, osg::StateAttribute::ON);
    ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    return xform;
}

// 偏差 HUD
static osg::Camera* createHud(int w, int h)
{
    auto* hud = new osg::Camera;
    hud->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    hud->setViewMatrix(osg::Matrix::identity());
    hud->setProjectionMatrix(osg::Matrix::ortho2D(0, w, 0, h));
    hud->setClearMask(GL_DEPTH_BUFFER_BIT);
    hud->setRenderOrder(osg::Camera::POST_RENDER, 10);

    auto* geode = new osg::Geode;
    hud->addChild(geode);

    osg::StateSet* ss = hud->getOrCreateStateSet();
    ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    ss->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);

    struct Item { const char* label; float val; const char* unit; };
    Item items[] = {
        {"\xE5\x89\x8D\xE5\x90\x8E", 12.3f, "mm"},
        {"\xE5\xB7\xA6\xE5\x8F\xB3", -5.1f, "mm"},
        {"\xE8\xBF\x9C\xE8\xBF\x91",  8.7f, "mm"},
        {"\xE4\xBF\xAF\xE4\xBB\xB0",  2.1f, "\xC2\xB0"},
        {"\xE5\x81\x8F\xE8\x88\xAA", -1.3f, "\xC2\xB0"},
        {"\xE7\xBF\xBB\xE6\xBB\x9A",  0.5f, "\xC2\xB0"},
    };

    float panelX = (float)w - 260.0f;
    float panelH = 30.0f;
    float startY = (float)h - 40.0f;

    for (int i = 0; i < 6; ++i) {
        float y = startY - i * panelH;

        auto* label = new osgText::Text;
        label->setText(items[i].label);
        label->setPosition(osg::Vec3(panelX, y, 0));
        label->setCharacterSize(16.0f);
        label->setColor(osg::Vec4(1, 1, 1, 1));
        geode->addDrawable(label);

        char buf[32];
        snprintf(buf, sizeof(buf), "%+.1f%s", items[i].val, items[i].unit);
        auto* val = new osgText::Text;
        val->setText(buf);
        val->setPosition(osg::Vec3(panelX + 60, y, 0));
        val->setCharacterSize(16.0f);
        float level = fabsf(items[i].val) / (i < 3 ? 30.0f : 10.0f);
        level = std::min(level, 1.0f);
        val->setColor(osg::Vec4(level, 1 - level, 0, 1));
        geode->addDrawable(val);

        for (int j = 0; j < 5; ++j) {
            float segLevel = j / 4.0f;
            float alpha = (segLevel <= level) ? 1.0f : 0.2f;
            float bx = panelX + 150 + j * 18;
            float by = y - 2;

            osg::ref_ptr<osg::Vec3Array> v = new osg::Vec3Array;
            v->push_back(osg::Vec3(bx, by, 0));
            v->push_back(osg::Vec3(bx + 16, by, 0));
            v->push_back(osg::Vec3(bx + 16, by + 14, 0));
            v->push_back(osg::Vec3(bx, by + 14, 0));

            osg::ref_ptr<osg::Vec4Array> c = new osg::Vec4Array;
            c->push_back(osg::Vec4(segLevel, 1 - segLevel, 0, alpha));

            auto* seg = new osg::Geometry;
            seg->setUseDisplayList(false);
            seg->setVertexArray(v);
            seg->setColorArray(c);
            seg->setColorBinding(osg::Geometry::BIND_OVERALL);
            seg->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS, 0, 4));
            geode->addDrawable(seg);
        }
    }
    return hud;
}

osg::Group* buildCalibScene(const std::string& stlPath)
{
    osg::ref_ptr<osg::Group> root = new osg::Group;

    // 标定板组：放大 2.2 + 绕 Z 顺时针 90° + Y+40 平移
    auto* boardGroup = new osg::MatrixTransform;
    boardGroup->setMatrix(osg::Matrix::translate(0.0f, 40.0f, 0.0f) *
                          osg::Matrix::rotate(osg::DegreesToRadians(-90.0f), osg::Vec3(0, 0, 1)) *
                          osg::Matrix::scale(3.3f, 3.3f, 3.3f));
    {
        auto* boardGeode = new osg::Geode;
        boardGeode->addDrawable(createBoardPlane());
        boardGeode->addDrawable(createGridLines());
        boardGroup->addChild(boardGeode);
        boardGroup->addChild(createMarkers());
    }
    root->addChild(boardGroup);

    // 扫描仪组：缩放 0.6 + 绕 Z 顺时针 90° + Y-220 平移（绿=目标 红=当前，STL 原始坐标）
    auto* scanGroup = new osg::MatrixTransform;
    scanGroup->setMatrix(osg::Matrix::translate(0.0f, -220.0f, 0.0f) *
                         osg::Matrix::rotate(osg::DegreesToRadians(-90.0f), osg::Vec3(0, 0, 1)) *
                         osg::Matrix::scale(0.6f, 0.6f, 0.6f));
    {
        auto* target = createPoseModel(stlPath, 0, 1, 0);   // 绿
        auto* current = createPoseModel(stlPath, 1, 0, 0);  // 红
        if (target)  scanGroup->addChild(target);
        if (current) scanGroup->addChild(current);
    }
    root->addChild(scanGroup);

    // 偏差 HUD
    root->addChild(createHud(1280, 720));

    return root.release();
}

} // namespace calib_display
