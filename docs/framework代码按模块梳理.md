# framework 代码按模块梳理

> 日期: 2026-08-08
> 目标: 将项目所有源码按 11 模块定义归类，明确每段代码归属哪个模块、当前在哪个目录、应放到哪个模块目录

---

## 当前代码分布

```
framework/
├── app/                     10 cpp + 14 h  ← 主程序（单体，大部分业务逻辑堆在这）
│   ├── stubs/               5 h（空桩）
│   └── resources/           UI资源
├── framework/               19 cpp + 28 h  ← 分层框架骨架
│   ├── data/                5 cpp + 8 h
│   ├── workflow/            6 cpp + 6 h
│   ├── service/             4 cpp + 5 h
│   ├── infra/               3 cpp + 3 h
│   ├── hal/                 0 cpp + 2 h
│   ├── common/              1 cpp + 1 h
│   ├── crosscut/            0 cpp + 1 h
│   ├── algorithm/           0 cpp + 1 h
│   ├── ui/                  0 cpp + 1 h
│   └── tests/               2 cpp
├── modules/
│   ├── 06_device_control/   3 cpp + 3 h    ← 唯一有代码的业务模块
│   └── 09_operatorlib/      84 cpp + 72 h + 17 cu  ← 算子库（唯一扎实的模块）
└── sdk/                                    ← SDK（空）
```

---

## 按模块归类

### 模块1 · 标定

| 文件 | 当前位置 | 类型 | 说明 |
|------|---------|------|------|
| `calib_workflow.cpp/h` | app/ | Workflow | 相机标定4步 + 激光标定 + JSON保存/加载 |
| `CalibrationWorkflow.cpp/h` | framework/workflow/ | Workflow框架 | 标定工作流基类 |
| `CalibDialog.cpp/h` | app/ | UI | 标定对话框（相机/激光标定入口） |
| `CalibDisplay.cpp/h` | app/ | UI/渲染 | 标定3D显示（扫描仪模型+标定板+姿态彩条） |
| `IntegrateTestDialog.cpp/h` | app/ | UI | 标定集成测试对话框 |
| `CalibStore.cpp/h` | framework/data/ | Data | 标定数据存储 |
| `calibration/` (32cpp+7cu) | modules/09_operatorlib/ | Algorithm | 16个标定算子（内参/外参/矫正/温度补偿/激光平面映射） |
| `camera_calib_workflow.h` | app/stubs/ | Stub桩 | 相机标定空桩 |
| `laser_calib_workflow.h` | app/stubs/ | Stub桩 | 激光标定空桩 |

**应移至**: `modules/01_calibration/`

---

### 模块2 · 扫描

| 文件 | 当前位置 | 类型 | 说明 |
|------|---------|------|------|
| `ScanWorkflow.cpp/h` | framework/workflow/ | Workflow框架 | 扫描工作流 |
| `Pipeline.cpp/h` | framework/workflow/ | Workflow框架 | 流水线（Stage概念） |
| `WorkflowContext.cpp/h` | framework/workflow/ | 共享 | 工作流上下文 |
| `IWorkflow.h` | framework/workflow/ | 共享接口 | 工作流接口 |
| `FrameBuffer.cpp/h` | framework/data/ | Data | 帧缓冲 |
| `RingBuffer.h` | framework/data/ | Data | 环形缓冲（模板） |
| `ScannerWindow.cpp/h` | app/ | UI | 扫描窗口（UI界面） |
| `core/` (36cpp+6cu) | modules/09_operatorlib/ | Algorithm | 标记点链+激光链+配准+融合算子 |
| `scanning/` (16cpp+4cu) | modules/09_operatorlib/ | Algorithm | mask_separation/laser_match_scan/融合/法线 |
| `scan_workflow.h` | app/stubs/ | Stub桩 | 扫描空桩 |

**应移至**: `modules/02_scanning/`（算子可在 09_operatorlib 保留，扫描模块引用）

---

### 模块3 · 渲染显示

| 文件 | 当前位置 | 类型 | 说明 |
|------|---------|------|------|
| `OSGWidget.cpp/h` | app/ | UI/渲染 | **核心渲染控件**（点云/网格/坐标轴/相机控制/套索/MSAA/深度测试） |
| `MainWindow.cpp/h`（部分） | app/ | UI | 3D视图区域布局（view3DArea）、文件管理菜单、标定/扫描切换 |
| `PointCloudBuffer.cpp/h` | framework/data/ | Data | 点云缓冲（渲染直读快照源） |
| `IPointCloudSink.h` | framework/data/ | Data接口 | 点云写入接口 |
| `resources.qrc + resources/` | app/ | UI资源 | 图标/QSS样式 |

**应移至**: `modules/03_rendering/`（OSGWidget + PointCloudBuffer + UI资源）

> **注意**: `OSGWidget.cpp` 同时承载了渲染（模块3）+ 编辑（模块5）功能，重构时需拆分。

---

### 模块4 · 后处理

| 文件 | 当前位置 | 类型 | 说明 |
|------|---------|------|------|
| `PostProcessWorkflow.cpp/h` | framework/workflow/ | Workflow框架 | 后处理工作流 |
| GBA/融合/法线算子 | modules/09_operatorlib/ | Algorithm | global_ba_cpu / laser_cloud_fuse_cuda / laser_cloud_normal_cuda（被模块4调用） |

**应移至**: `modules/04_postprocessing/`（目前空，只有 Workflow 框架在 framework/workflow/）

---

### 模块5 · 编辑

| 文件 | 当前位置 | 类型 | 说明 |
|------|---------|------|------|
| OSGWidget 中的编辑相关代码 | app/OSGWidget.cpp | UI/逻辑 | 套索选择(lasso mode)、点删除(deleteSelectedPoints)、撤销(undoDelete)、高亮(highlightSelectedPoints) |

**应移至**: `modules/05_editing/`（从 OSGWidget 拆分编辑逻辑，建 EditCommand/EditCommandStack）

> **当前编辑代码内嵌在 OSGWidget**，无独立文件。重构时需提取。

---

### 模块6 · 文件管理

| 文件 | 当前位置 | 类型 | 说明 |
|------|---------|------|------|
| `file_io.cpp/h` | app/ | Service/IO | PLY/STL/OBJ/PCD/XYZ 导入导出 + 标志点JSON |
| `MainWindow.cpp`（部分） | app/ | UI | 文件管理菜单（导入/导出/进度框） |

**应移至**: `modules/06_fileio/`（file_io.cpp/h + 导入导出UI）

---

### 模块7 · 会话管理

| 文件 | 当前位置 | 类型 | 说明 |
|------|---------|------|------|
| `SessionService.cpp/h` | framework/service/ | Service | 会话服务（框架壳） |
| `StateMachine.cpp/h` | framework/service/ | Service | 状态机（框架壳） |
| `IState.h` | framework/service/ | Service接口 | 状态接口 |
| `AppContext.cpp/h` | app/ | 应用上下文 | 组件装配/生命周期/启动/关闭 |

**应移至**: `modules/07_session/`（SessionService + StateMachine + AppContext中会话相关部分）

---

### 模块8 · 设备管理

| 文件 | 当前位置 | 类型 | 说明 |
|------|---------|------|------|
| `CameraControl.cpp/h` | modules/06_device_control/ | HAL | 相机控制 |
| `MCUDriver.cpp/h` | modules/06_device_control/ | HAL | MCU串口驱动 |
| `HardwareMonitor.cpp/h` | modules/06_device_control/ | HAL | 硬件监控 |
| `IMCU.h` | framework/hal/ | HAL接口 | MCU抽象接口 |
| `IScannerCamera.h` | framework/hal/ | HAL接口 | 相机抽象接口 |
| `DeviceStateCache.cpp/h` | framework/data/ | Data | 设备状态缓存 |
| `FaultHandler.cpp/h` | framework/service/ | Service | 故障处理 |
| `CameraControl.h` | app/stubs/ | Stub桩 | 设备控制桩 |
| `LEADSCANSeries.h` | app/stubs/ | Stub桩 | 扫描仪系列桩 |

**应移至**: `modules/08_devicemgmt/`（当前代码在 06_device_control，需改名+合并 hal/ + DeviceStateCache + FaultHandler）

---

### 模块9 · 算子库

| 文件 | 当前位置 | 类型 | 说明 |
|------|---------|------|------|
| `modules/09_operatorlib/` 全部 | modules/09_operatorlib/ | Algorithm | **84 cpp + 72 h + 17 cu**（calibration + core + scanning） |

**当前已在正确位置** ✅。这是唯一代码量和位置都对的模块。

---

### 模块10 · 可观测性

| 文件 | 当前位置 | 类型 | 说明 |
|------|---------|------|------|
| spdlog 配置 | app/main.cpp | Infra | `spdlog::set_level(info)` |
| 散落的 spdlog 调用 | 各文件 | Infra | `spdlog::info(...)` 等 |

**应新建**: `modules/10_observability/`（统一日志配置、PerfCounters、崩溃捕获）

> 当前无可观测性独立代码，仅 spdlog 基础调用散落各处。

---

### 模块11 · 安装部署

| 文件 | 当前位置 | 类型 | 说明 |
|------|---------|------|------|
| `build_cuda.bat` | 根目录 | 脚本 | CMake构建 |
| `run.bat` | 根目录 | 脚本 | 运行 |
| `copy_dlls.bat` | app/ | 脚本 | DLL拷贝（POST_BUILD） |
| `main.cpp` | app/ | 入口 | 程序入口（QApplication + MSAA + AppContext初始化） |

**应新建**: `modules/11_deploy/`（安装包脚本 + 首配向导）

---

## 共享基础设施（不属于任何模块）

这些代码是跨模块共享的基础设施，不应放入单个模块目录：

| 文件 | 当前位置 | 说明 |
|------|---------|------|
| `EventBus` | framework/infra/ | 事件总线（全项目用） |
| `GpuRuntime` | framework/infra/ | GPU运行时 |
| `Scheduler` | framework/infra/ | 线程调度 |
| `Watchdog` | framework/infra/ | 心跳看门狗 |
| `ParameterManager.cpp/h` | framework/service/ | 参数管理（配置注入） |
| `common/` | framework/common/ | 通用工具 |
| `crosscut/IAuth.h, IConfig.h` | framework/crosscut/ | 横切接口（权限/配置） |
| `CMakeLists.txt` 顶层 | 根目录 | 依赖配置（OpenCV/Qt/OSG/CUDA/Eigen） |
| `cmake/` | 根目录 | CMake模块（Version.h.in/CompilerSettings） |

> 架构文档定义: SDK、用户权限/鉴权、配置管理 是"框架结构元素"，不计入 11 模块。

---

## 重构建议（如需模块化）

### 优先级1: 先改目录名（不改代码位置）

```
当前                          →  应为
modules/05_file_io (空)       →  modules/05_editing
modules/06_device_control     →  modules/06_fileio (file_io.cpp搬入)
modules/07_ui_main (空)       →  modules/07_session
modules/08_license (空)       →  modules/08_devicemgmt (device_control搬入)
modules/10_edit (空)          →  modules/10_observability
modules/11_marker (空)        →  modules/11_deploy
```

### 优先级2: 把 app/ 里的代码搬到对应模块

```
app/calib_workflow.cpp/h      → modules/01_calibration/
app/CalibDialog.cpp/h         → modules/01_calibration/
app/CalibDisplay.cpp/h        → modules/01_calibration/
app/ScannerWindow.cpp/h       → modules/02_scanning/
app/OSGWidget.cpp/h           → modules/03_rendering/ (编辑部分拆到 05_editing)
app/file_io.cpp/h             → modules/06_fileio/
app/MainWindow.cpp/h          → 保留在 app/（主窗口组装各模块UI）
app/AppContext.cpp/h          → 保留在 app/ 或移入 framework/infra/
app/main.cpp                  → 保留在 app/（程序入口）
```

### 优先级3: 把 framework/ 里的代码归到模块或保留共享层

```
framework/workflow/Calibration*  → modules/01_calibration/
framework/workflow/Scan*         → modules/02_scanning/
framework/workflow/PostProcess*  → modules/04_postprocessing/
framework/workflow/Pipeline*     → 保留 framework/infra/（共享流水线框架）
framework/data/CalibStore        → modules/01_calibration/
framework/data/FrameBuffer       → modules/02_scanning/
framework/data/PointCloudBuffer  → modules/03_rendering/
framework/data/DeviceStateCache  → modules/08_devicemgmt/
framework/service/Session*       → modules/07_session/
framework/service/FaultHandler   → modules/08_devicemgmt/
framework/hal/*                  → modules/08_devicemgmt/
```

> **注意**: 物理移动代码需同步修改 CMakeLists.txt（各模块的 add_subdirectory + target_link_libraries）和 include 路径。建议分步进行，每步编译验证。
