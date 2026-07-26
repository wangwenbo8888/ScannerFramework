# 算子合并检查清单

## 📋 合并前检查

### 符号检查
- [ ] 所有类名/函数名在命名空间内唯一
- [ ] 无全局命名空间的符号
- [ ] 检查导出符号：`dumpbin /exports Algorithm.dll`

### 依赖检查
- [ ] 第三方库版本一致（OpenCV、PCL、Eigen等）
- [ ] 编译选项一致（/MT vs /MD、优化级别等）
- [ ] 预处理定义一致（BUILD_CUDA、WIN32等）

### 内存检查
- [ ] 无内存泄漏（使用Valgrind/DrMemory）
- [ ] GPU内存正确释放（cuda-memcheck）
- [ ] 统一内存池配置

### 链接检查
- [ ] 无循环依赖
- [ ] 所有符号都有定义
- [ ] 链接顺序正确

### 头文件检查
- [ ] 无绝对路径
- [ ] include guard正确
- [ ] 公共头文件最小化

---

## 🔴 高优先级问题

### 1. 符号冲突（最重要）

独立编译时可能存在的问题：

```cpp
// 算子A
namespace algo_a {
    struct Result { int x; };  // Result
}

// 算子B  
namespace algo_b {
    struct Result { int y; };  // 同名但不同结构
}

// 合并后：链接器看到两个 Result，冲突！
```

**解决方案**：

```cpp
// ✅ 方案1：唯一命名空间
namespace calibration::mask_extract {
    struct Result { ... };
}

namespace calibration::region_analyze {
    struct Result { ... };
}

// ✅ 方案2：模块前缀
struct MaskExtractResult { ... };
struct RegionAnalyzeResult { ... };
```

---

### 2. 导出符号控制

**问题**：每个算子独立编译时可能导出过多符号

```cpp
// ❌ 错误：导出所有符号
class Algorithm {
    // 内部辅助函数也被导出
    void internalHelper() { ... }  // 不应导出
};

// ✅ 正确：只导出公共接口
class ALGORITHM_API Algorithm {
    // 只有 public + ALGORITHM_API 的才导出
    Result process(const Input& in);
    
private:
    // 私有成员不导出
    void internalHelper() { ... }
};
```

**检查方法**：

```bash
# Windows：检查导出符号
dumpbin /exports Algorithm.dll | findstr "YourClass"

# Linux：检查符号表
nm -D libalgorithm.so | grep "T " | c++filt
```

---

### 3. 头文件组织

**问题**：独立编译时头文件路径不一致

```cpp
// ❌ 错误：绝对路径
#include "E:/workfold/20260509intergrate/Algorithm/calib/mask_extract.h"

// ❌ 错误：深层相对路径
#include "../../../../../calib/mask_extract.h"

// ✅ 正确：统一include根目录
// 构建时设置：-I Algorithm/calib
#include "mask_extract.h"  // 相对于calib目录
```

**建议的目录结构**：

```
Algorithm/
├── include/              # 公共头文件（对外暴露）
│   ├── common/
│   │   └── operator_base.h
│   ├── LaserCalib/
│   │   ├── mask_extract.h
│   │   └── region_analyze.h
│   └── CameraCalib/
│       └── inverse_distort.h
│
├── src/                  # 实现文件（不对外暴露）
│   ├── LaserCalib/
│   │   ├── mask_extract.cpp
│   │   └── mask_extract_impl.cu
│   └── CameraCalib/
│       └── inverse_distort.cpp
│
└── Algorithm.vcxproj
```

---

## 🟡 中优先级问题

### 4. 依赖库版本一致性

**问题**：各算子可能依赖不同版本的第三方库

```cpp
// 算子A依赖 OpenCV 4.8
// 算子B依赖 OpenCV 4.13
// 合并后：运行时加载哪个版本？
```

**解决方案**：

```cpp
// ✅ 统一版本管理
// CMakeLists.txt
find_package(OpenCV 4.13 REQUIRED)  # 统一版本
target_link_libraries(algorithm PRIVATE ${OpenCV_LIBS})

// 或者在vcxproj中统一配置
```

---

### 5. 链接顺序依赖

**问题**：库之间有循环依赖

```cpp
// mask_extract 依赖 region_analyze
// region_analyze 依赖 laser_label
// laser_label 依赖 mask_extract  ← 循环！
```

**解决方案**：

```cpp
// ✅ 方案1：重新设计依赖关系
mask_extract → region_analyze → laser_label
（单向依赖）

// ✅ 方案2：合并到同一库（内部不链接）
// 所有算子都在Algorithm.dll内部，不互相链接
```

---

### 6. CUDA代码分离

**问题**：GPU算子有`.cu`文件，需要特殊处理

```cpp
// ❌ 错误：.cu文件直接参与编译
// 链接器不理解CUDA PTX代码

// ✅ 正确：CUDA代码单独编译成对象文件
// 1. 编译.cu → .obj (nvcc)
// 2. 编译.cpp → .obj (cl.exe)
// 3. 链接所有.obj → Algorithm.dll (link.exe)
```

**vcxproj配置示例**：

```xml
<!-- .cu文件 -->
<CudaCompile Include="mask_extract_impl.cu">
  <CodeGeneration>compute_52,sm_52</CodeGeneration>
</CudaCompile>

<!-- .cpp文件 -->
<ClCompile Include="mask_extract.cpp" />

<!-- 统一链接 -->
<Link>
  <AdditionalDependencies>cuda.lib;cudart_static.lib</AdditionalDependencies>
</Link>
```

---

### 7. 内存管理统一

**问题**：各算子可能使用不同的内存分配器

```cpp
// 算子A使用 cv::cuda::GpuMat（CUDA内存）
// 算子B使用 std::vector（系统内存）
// 算子C使用自定义内存池
```

**解决方案**：

```cpp
// ✅ 统一内存管理
class GpuMemoryPool {
    // 所有GPU算子共用
    cv::cuda::GpuMat allocate(int rows, int cols, int type);
    void deallocate(cv::cuda::GpuMat& mat);
};

// 在算子中使用
class MaskExtract {
    GpuMemoryPool* pool_;  // 注入统一内存池
};
```

---

## 🟢 低优先级问题

### 8. 编译选项一致性

```cpp
// ❌ 问题：各算子编译选项不同
// 算子A: /O2 (优化速度)
// 算子B: /Od (调试)
// 合并后：运行时行为不一致

// ✅ 解决：统一编译选项
// vcxproj中设置
<ClCompile>
  <Optimization>MaxSpeed</Optimization>
  <RuntimeLibrary>MultiThreadedDLL</RuntimeLibrary>
  <PreprocessorDefinitions>BUILD_CUDA=1;%(PreprocessorDefinitions)</PreprocessorDefinitions>
</ClCompile>
```

---

### 9. 异常处理统一

```cpp
// ❌ 问题：异常处理方式不一致
// 算子A使用 try-catch
// 算子B使用错误码
// 算子C使用 std::optional

// ✅ 解决：统一错误处理
class Result {
    bool success_;
    std::string message_;
    QualityFlag flag_;
    
    static Result ok();
    static Result fail(const std::string& msg);
};
```

---

### 10. 版本管理

```cpp
// ✅ 添加版本信息
// Algorithm.h
#define ALGORITHM_VERSION_MAJOR 1
#define ALGORITHM_VERSION_MINOR 0
#define ALGORITHM_VERSION_PATCH 0

// 算子注册时携带版本
struct OperatorInfo {
    const char* name;
    int version;
    OperatorType type;
};

// 导出函数
extern "C" ALGORITHM_API OperatorInfo* getAlgorithmInfo();
```

---

## 🔧 推荐的构建流程

```cmake
# CMakeLists.txt 示例
cmake_minimum_required(VERSION 3.20)
project(Algorithm)

# 1. 统一依赖
find_package(CUDA REQUIRED)
find_package(OpenCV 4.13 REQUIRED)
find_package(Eigen3 REQUIRED)

# 2. 收集所有源文件
file(GLOB_RECURSE CPP_SOURCES "src/*.cpp")
file(GLOB_RECURSE CU_SOURCES "src/*.cu")

# 3. 编译CUDA文件
cuda_add_library(algorithm SHARED
    ${CPP_SOURCES}
    ${CU_SOURCES}
)

# 4. 统一链接
target_link_libraries(algorithm
    PRIVATE
        ${OpenCV_LIBS}
        CUDA::cudart
        Eigen3::Eigen
)

# 5. 设置公共头文件
target_include_directories(algorithm
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# 6. 导出符号控制
set_target_properties(algorithm PROPERTIES
    WINDOWS_EXPORT_ALL_SYMBOLS OFF
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
)
```

---

## 📊 检查优先级总结

| 优先级 | 问题 | 影响 | 检查方法 |
|--------|------|------|----------|
| 🔴 高 | 符号冲突 | 链接失败 | dumpbin /exports |
| 🔴 高 | 导出符号控制 | 符号污染 | 检查ALGORITHM_API宏 |
| 🔴 高 | 头文件组织 | 编译失败 | 检查include路径 |
| 🟡 中 | 依赖库版本 | 运行时错误 | 检查CMake/vcxproj |
| 🟡 中 | 链接顺序 | 链接失败 | 检查链接器日志 |
| 🟡 中 | CUDA分离 | 编译失败 | 检查nvcc配置 |
| 🟡 中 | 内存管理 | 内存泄漏 | Valgrind/cuda-memcheck |
| 🟢 低 | 编译选项 | 行为不一致 | 检查编译日志 |
| 🟢 低 | 异常处理 | 错误处理混乱 | 代码审查 |
| 🟢 低 | 版本管理 | 兼容性问题 | 检查版本宏 |
