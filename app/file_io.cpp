#include "file_io.h"

#include <osg/Vec3>
#include <fstream>
#include <sstream>
#include <cstring>
#include <cstdio>
#include <cmath>

namespace file_io {

// ============================================================================
// 点云
// ============================================================================

bool importPointCloud(const std::string& filepath,
                     std::vector<osg::Vec3>& points,
                     std::vector<osg::Vec3>* normals)
{
    // 根据扩展名分派
    auto ext = filepath.substr(filepath.find_last_of('.') + 1);
    if (ext == "ply") return importPLY(filepath, points, normals);
    if (ext == "xyz" || ext == "txt") return importXYZ(filepath, points, normals);
    if (ext == "pcd") return importPCD(filepath, points, normals);
    // 默认按 xyz 处理
    return importXYZ(filepath, points, normals);
}

bool exportPointCloud(const std::string& filepath,
                      const std::vector<osg::Vec3>& points,
                      const std::vector<osg::Vec3>* normals)
{
    auto ext = filepath.substr(filepath.find_last_of('.') + 1);
    if (ext == "ply") return exportPLY(filepath, points, normals);
    if (ext == "pcd") return exportPCD(filepath, points, normals);
    // 默认 xyz
    return exportXYZ(filepath, points, normals);
}

// --- XYZ/TXT ---
bool importXYZ(const std::string& filepath,
               std::vector<osg::Vec3>& points,
               std::vector<osg::Vec3>* normals)
{
    std::ifstream f(filepath);
    if (!f.is_open()) return false;
    points.clear();
    if (normals) normals->clear();
    double x, y, z, nx, ny, nz;
    std::string line;
    while (std::getline(f, line)) {
        std::istringstream ss(line);
        if (ss >> x >> y >> z) {
            points.emplace_back((float)x, (float)y, (float)z);
            if (normals && (ss >> nx >> ny >> nz))
                normals->emplace_back((float)nx, (float)ny, (float)nz);
        }
    }
    return !points.empty();
}

bool exportXYZ(const std::string& filepath,
               const std::vector<osg::Vec3>& points,
               const std::vector<osg::Vec3>* normals)
{
    std::ofstream f(filepath);
    if (!f.is_open()) return false;
    f << std::fixed;
    f.precision(5);
    for (size_t i = 0; i < points.size(); ++i) {
        f << points[i].x() << " " << points[i].y() << " " << points[i].z();
        if (normals && i < normals->size())
            f << " " << (*normals)[i].x() << " " << (*normals)[i].y() << " " << (*normals)[i].z();
        f << "\n";
    }
    return true;
}

// --- PLY ---
bool importPLY(const std::string& filepath,
               std::vector<osg::Vec3>& points,
               std::vector<osg::Vec3>* normals)
{
    std::ifstream f(filepath, std::ios::binary);
    if (!f.is_open()) return false;
    std::string line;
    int vertexCount = 0;
    bool hasNormal = false;
    bool binary = false;

    // 读 header
    while (std::getline(f, line)) {
        if (line.find("format binary") != std::string::npos) binary = true;
        if (line.find("element vertex") != std::string::npos)
            sscanf(line.c_str(), "element vertex %d", &vertexCount);
        if (line.find("property float nx") != std::string::npos) hasNormal = true;
        if (line == "end_header" || line == "end_header\r") break;
    }

    if (vertexCount <= 0) return false;
    points.clear();
    if (normals) normals->clear();
    points.reserve(vertexCount);
    if (normals) normals->reserve(vertexCount);

    if (binary) {
        for (int i = 0; i < vertexCount; ++i) {
            float xyz[3];
            f.read(reinterpret_cast<char*>(xyz), 12);
            points.emplace_back(xyz[0], xyz[1], xyz[2]);
            if (hasNormal) {
                float n[3];
                f.read(reinterpret_cast<char*>(n), 12);
                if (normals) normals->emplace_back(n[0], n[1], n[2]);
            }
        }
    } else {
        for (int i = 0; i < vertexCount; ++i) {
            float x, y, z;
            f >> x >> y >> z;
            points.emplace_back(x, y, z);
            if (hasNormal) {
                float nx, ny, nz;
                f >> nx >> ny >> nz;
                if (normals) normals->emplace_back(nx, ny, nz);
            }
        }
    }
    return !points.empty();
}

bool exportPLY(const std::string& filepath,
               const std::vector<osg::Vec3>& points,
               const std::vector<osg::Vec3>* normals)
{
    std::ofstream f(filepath, std::ios::binary);
    if (!f.is_open()) return false;
    bool hasN = normals && normals->size() == points.size();
    // header
    f << "ply\n";
    f << "format ascii 1.0\n";
    f << "element vertex " << points.size() << "\n";
    f << "property float x\nproperty float y\nproperty float z\n";
    if (hasN)
        f << "property float nx\nproperty float ny\nproperty float nz\n";
    f << "end_header\n";
    // data
    f << std::fixed;
    f.precision(5);
    for (size_t i = 0; i < points.size(); ++i) {
        f << points[i].x() << " " << points[i].y() << " " << points[i].z();
        if (hasN)
            f << " " << (*normals)[i].x() << " " << (*normals)[i].y() << " " << (*normals)[i].z();
        f << "\n";
    }
    return true;
}

// --- PCD ---
bool importPCD(const std::string& filepath,
               std::vector<osg::Vec3>& points,
               std::vector<osg::Vec3>* normals)
{
    std::ifstream f(filepath, std::ios::binary);
    if (!f.is_open()) return false;
    std::string line;
    int points_count = 0;
    std::string data_type = "ascii";

    while (std::getline(f, line)) {
        if (line.find("POINTS") != std::string::npos)
            sscanf(line.c_str(), "POINTS %d", &points_count);
        if (line.find("DATA") != std::string::npos)
            data_type = line.substr(5);
        if (line == "DATA" || data_type != "") break;
    }

    if (points_count <= 0) return false;
    points.clear();
    points.reserve(points_count);

    if (data_type == "binary") {
        for (int i = 0; i < points_count; ++i) {
            float xyz[3];
            f.read(reinterpret_cast<char*>(xyz), 12);
            points.emplace_back(xyz[0], xyz[1], xyz[2]);
        }
    } else {
        for (int i = 0; i < points_count; ++i) {
            float x, y, z;
            if (!(f >> x >> y >> z)) break;
            points.emplace_back(x, y, z);
        }
    }
    return !points.empty();
}

bool exportPCD(const std::string& filepath,
               const std::vector<osg::Vec3>& points,
               const std::vector<osg::Vec3>* normals)
{
    std::ofstream f(filepath);
    if (!f.is_open()) return false;
    f << "# .PCD v0.7 - Point Cloud Data file format\n";
    f << "VERSION 0.7\n";
    f << "FIELDS x y z\n";
    f << "SIZE 4 4 4\n";
    f << "TYPE F F F\n";
    f << "COUNT 1 1 1\n";
    f << "WIDTH " << points.size() << "\n";
    f << "HEIGHT 1\n";
    f << "VIEWPOINT 0 0 0 1 0 0 0\n";
    f << "POINTS " << points.size() << "\n";
    f << "DATA ascii\n";
    f << std::fixed;
    f.precision(5);
    for (const auto& p : points)
        f << p.x() << " " << p.y() << " " << p.z() << "\n";
    return true;
}

// ============================================================================
// 网格
// ============================================================================

bool importMesh(const std::string& filepath, MeshData& mesh)
{
    auto ext = filepath.substr(filepath.find_last_of('.') + 1);
    if (ext == "stl") return importSTL(filepath, mesh);
    if (ext == "obj") return importOBJ(filepath, mesh);
    return false;
}

bool exportMesh(const std::string& filepath, const MeshData& mesh)
{
    auto ext = filepath.substr(filepath.find_last_of('.') + 1);
    if (ext == "stl") return exportSTL(filepath, mesh);
    if (ext == "obj") return exportOBJ(filepath, mesh);
    return false;
}

// --- STL (binary) ---
bool importSTL(const std::string& filepath, MeshData& mesh)
{
    FILE* f = fopen(filepath.c_str(), "rb");
    if (!f) return false;

    char header[80];
    if (fread(header, 1, 80, f) != 80) { fclose(f); return false; }

    unsigned int numTris = 0;
    if (fread(&numTris, 4, 1, f) != 1) { fclose(f); return false; }
    if (numTris == 0 || numTris > 100000000) { fclose(f); return false; }

    mesh.vertices.clear();
    mesh.indices.clear();
    mesh.vertices.reserve(numTris * 3);
    mesh.indices.reserve(numTris * 3);

    for (unsigned int i = 0; i < numTris; ++i) {
        float n[3], v[9];
        unsigned short attr;
        if (fread(n, 4, 3, f) != 3) break;
        if (fread(v, 4, 9, f) != 9) break;
        if (fread(&attr, 2, 1, f) != 1) break;

        unsigned int base = (unsigned int)mesh.vertices.size();
        mesh.vertices.emplace_back(v[0], v[1], v[2]);
        mesh.vertices.emplace_back(v[3], v[4], v[5]);
        mesh.vertices.emplace_back(v[6], v[7], v[8]);
        mesh.indices.push_back(base);
        mesh.indices.push_back(base + 1);
        mesh.indices.push_back(base + 2);
    }
    fclose(f);
    return !mesh.vertices.empty();
}

bool exportSTL(const std::string& filepath, const MeshData& mesh)
{
    FILE* f = fopen(filepath.c_str(), "wb");
    if (!f) return false;

    char header[80] = {0};
    memcpy(header, "binary STL", 10);
    fwrite(header, 1, 80, f);

    unsigned int numTris = (unsigned int)(mesh.indices.size() / 3);
    fwrite(&numTris, 4, 1, f);

    for (unsigned int i = 0; i < numTris; ++i) {
        const auto& v0 = mesh.vertices[mesh.indices[i * 3]];
        const auto& v1 = mesh.vertices[mesh.indices[i * 3 + 1]];
        const auto& v2 = mesh.vertices[mesh.indices[i * 3 + 2]];

        // 面法线
        osg::Vec3 e1 = v1 - v0;
        osg::Vec3 e2 = v2 - v0;
        osg::Vec3 n = e1 ^ e2;
        n.normalize();
        float fn[3] = { n.x(), n.y(), n.z() };
        fwrite(fn, 4, 3, f);

        float v[9] = {
            v0.x(), v0.y(), v0.z(),
            v1.x(), v1.y(), v1.z(),
            v2.x(), v2.y(), v2.z()
        };
        fwrite(v, 4, 9, f);

        unsigned short attr = 0;
        fwrite(&attr, 2, 1, f);
    }
    fclose(f);
    return true;
}

// --- OBJ ---
bool importOBJ(const std::string& filepath, MeshData& mesh)
{
    std::ifstream f(filepath);
    if (!f.is_open()) return false;
    mesh.vertices.clear();
    mesh.indices.clear();

    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        std::string type;
        ss >> type;
        if (type == "v") {
            float x, y, z;
            ss >> x >> y >> z;
            mesh.vertices.emplace_back(x, y, z);
        } else if (type == "f") {
            int idx[3];
            for (int k = 0; k < 3; ++k) {
                std::string token;
                ss >> token;
                idx[k] = std::stoi(token) - 1;  // OBJ 从1开始
                if (idx[k] < 0) idx[k] += (int)mesh.vertices.size() + 1;  // 负索引
            }
            mesh.indices.push_back((unsigned int)idx[0]);
            mesh.indices.push_back((unsigned int)idx[1]);
            mesh.indices.push_back((unsigned int)idx[2]);
        }
    }
    return !mesh.vertices.empty();
}

bool exportOBJ(const std::string& filepath, const MeshData& mesh)
{
    std::ofstream f(filepath);
    if (!f.is_open()) return false;
    f << std::fixed;
    f.precision(5);
    for (const auto& v : mesh.vertices)
        f << "v " << v.x() << " " << v.y() << " " << v.z() << "\n";
    for (size_t i = 0; i < mesh.indices.size(); i += 3)
        f << "f " << mesh.indices[i] + 1 << " " << mesh.indices[i + 1] + 1 << " " << mesh.indices[i + 2] + 1 << "\n";
    return true;
}

// ============================================================================
// 标志点 (JSON)
// ============================================================================

bool importMarkers(const std::string& filepath, std::vector<osg::Vec3>& markers)
{
    std::ifstream f(filepath);
    if (!f.is_open()) return false;
    markers.clear();
    std::string line;
    while (std::getline(f, line)) {
        // 兼容简单 JSON 数组格式: [x, y, z] 或 x y z
        float x, y, z;
        // 去掉 [ ] ,
        std::string clean;
        for (char c : line) {
            if (c != '[' && c != ']' && c != ',') clean += c;
            else clean += ' ';
        }
        std::istringstream ss(clean);
        if (ss >> x >> y >> z)
            markers.emplace_back(x, y, z);
    }
    return !markers.empty();
}

bool exportMarkers(const std::string& filepath, const std::vector<osg::Vec3>& markers)
{
    std::ofstream f(filepath);
    if (!f.is_open()) return false;
    f << "[\n";
    f << std::fixed;
    f.precision(5);
    for (size_t i = 0; i < markers.size(); ++i) {
        f << "  [" << markers[i].x() << ", " << markers[i].y() << ", " << markers[i].z() << "]";
        if (i + 1 < markers.size()) f << ",";
        f << "\n";
    }
    f << "]\n";
    return true;
}

} // namespace file_io
