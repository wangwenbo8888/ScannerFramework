cmake_minimum_required(VERSION 3.24)

# CMP0092: MSVC 不自动添加默认警告标志（防止 /W3→/W4 泄漏到 CUDA）
if(POLICY CMP0092)
    cmake_policy(SET CMP0092 NEW)
endif()

project(ScannerFramework VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(MSVC)
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:/utf-8>)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
endif()

# ============================================================================
# 输出目录
# ============================================================================
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# ============================================================================
# 编译选项
# ============================================================================
option(ENABLE_CUDA "Enable CUDA support" ON)
option(BUILD_TESTS "Build tests" ON)

include(cmake/CompilerSettings.cmake)

# ============================================================================
# 第三方依赖（FetchContent）
# ============================================================================
include(FetchContent)

set(JMW_GH_MIRROR "" CACHE STRING "GitHub mirror prefix (empty = direct)")

find_package(spdlog 1.15 QUIET)
if(NOT spdlog_FOUND)
    FetchContent_Declare(spdlog URL ${JMW_GH_MIRROR}https://github.com/gabime/spdlog/archive/refs/tags/v1.15.1.tar.gz)
    FetchContent_MakeAvailable(spdlog)
endif()

find_package(nlohmann_json 3.11.3 QUIET)
if(NOT nlohmann_json_FOUND)
    FetchContent_Declare(nlohmann_json URL ${JMW_GH_MIRROR}https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
    FetchContent_MakeAvailable(nlohmann_json)
endif()

if(BUILD_TESTS)
    FetchContent_Declare(googletest URL ${JMW_GH_MIRROR}https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz)
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
    enable_testing()
endif()

# ============================================================================
# OpenCV
# ============================================================================
set(OpenCV_DIR "F:/opencv4.13/install/x64/vc17/lib" CACHE PATH "OpenCV")
cmake_policy(SET CMP0146 OLD)
find_package(OpenCV 4.13 REQUIRED COMPONENTS core calib3d imgproc imgcodecs)

# 自定义 INTERFACE 库包装 OpenCV（解决 CMake 3.31 + OpenCV CUDA 策略冲突）
set(_OcvDir "${OpenCV_DIR}")
add_library(ScannerCV INTERFACE)
target_include_directories(ScannerCV INTERFACE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(ScannerCV INTERFACE
    $<$<CONFIG:Debug>:${_OcvDir}/opencv_flann4130d.lib>$<$<NOT:$<CONFIG:Debug>>:${_OcvDir}/opencv_flann4130.lib>
    $<$<CONFIG:Debug>:${_OcvDir}/opencv_core4130d.lib>$<$<NOT:$<CONFIG:Debug>>:${_OcvDir}/opencv_core4130.lib>
    $<$<CONFIG:Debug>:${_OcvDir}/opencv_imgproc4130d.lib>$<$<NOT:$<CONFIG:Debug>>:${_OcvDir}/opencv_imgproc4130.lib>
    $<$<CONFIG:Debug>:${_OcvDir}/opencv_features2d4130d.lib>$<$<NOT:$<CONFIG:Debug>>:${_OcvDir}/opencv_features2d4130.lib>
    $<$<CONFIG:Debug>:${_OcvDir}/opencv_calib3d4130d.lib>$<$<NOT:$<CONFIG:Debug>>:${_OcvDir}/opencv_calib3d4130.lib>
    $<$<CONFIG:Debug>:${_OcvDir}/opencv_imgcodecs4130d.lib>$<$<NOT:$<CONFIG:Debug>>:${_OcvDir}/opencv_imgcodecs4130.lib>
    $<$<CONFIG:Debug>:${_OcvDir}/opencv_cudev4130d.lib>$<$<NOT:$<CONFIG:Debug>>:${_OcvDir}/opencv_cudev4130.lib>
)
set(OpenCV_LIBS ScannerCV)

# ============================================================================
# CUDA
# ============================================================================
option(ENABLE_CUDA "Enable CUDA support" ON)
if(ENABLE_CUDA)
    set(BUILD_CUDA ON CACHE BOOL "Build CUDA operators" FORCE)
    set(CMAKE_CUDA_ARCHITECTURES 89)
    set(CMAKE_CUDA_STANDARD 17)
    set(CMAKE_CUDA_DEPENDENCIES_USE_COMPILER OFF)
    string(APPEND CMAKE_CUDA_FLAGS " --extended-lambda -Xcompiler=/utf-8 -DFMT_UNICODE=0")
    enable_language(CUDA)
    find_package(CUDAToolkit 12.6 REQUIRED)
    # OpenCV CUDA 模块
    target_link_libraries(ScannerCV INTERFACE
        $<$<CONFIG:Debug>:${_OcvDir}/opencv_cudaarithm4130d.lib>$<$<NOT:$<CONFIG:Debug>>:${_OcvDir}/opencv_cudaarithm4130.lib>
        $<$<CONFIG:Debug>:${_OcvDir}/opencv_cudawarping4130d.lib>$<$<NOT:$<CONFIG:Debug>>:${_OcvDir}/opencv_cudawarping4130.lib>
        $<$<CONFIG:Debug>:${_OcvDir}/opencv_cudaimgproc4130d.lib>$<$<NOT:$<CONFIG:Debug>>:${_OcvDir}/opencv_cudaimgproc4130.lib>
        $<$<CONFIG:Debug>:${_OcvDir}/opencv_cudafilters4130d.lib>$<$<NOT:$<CONFIG:Debug>>:${_OcvDir}/opencv_cudafilters4130.lib>
    )
    message(STATUS "CUDA enabled: ${CUDAToolkit_VERSION} arch=${CMAKE_CUDA_ARCHITECTURES}")
endif()

# ============================================================================
# Qt5
# ============================================================================
set(CMAKE_PREFIX_PATH "C:/Qt/Qt5.15.2/5.15.2/msvc2019_64" ${CMAKE_PREFIX_PATH})
find_package(Qt5 REQUIRED COMPONENTS Widgets OpenGL Svg SerialPort)

# ============================================================================
# OpenSceneGraph
# ============================================================================
set(OSG_ROOT "F:/osg3.6.5/install" CACHE PATH "OSG root")
find_path(OSG_INCLUDE_DIR osg/Group PATHS "${OSG_ROOT}/include")
find_library(OSG_LIBRARY osg PATHS "${OSG_ROOT}/lib")
find_library(OSGDB_LIBRARY osgDB PATHS "${OSG_ROOT}/lib")
find_library(OSGGA_LIBRARY osgGA PATHS "${OSG_ROOT}/lib")
find_library(OSGUTIL_LIBRARY osgUtil PATHS "${OSG_ROOT}/lib")
find_library(OSGVIEWER_LIBRARY osgViewer PATHS "${OSG_ROOT}/lib")
find_library(OSGTEXT_LIBRARY osgText PATHS "${OSG_ROOT}/lib")
find_library(OPENTHREADS_LIBRARY OpenThreads PATHS "${OSG_ROOT}/lib")

if(NOT OSG_LIBRARY)
    message(FATAL_ERROR "OpenSceneGraph not found at ${OSG_ROOT}")
endif()

add_library(OSG::osg IMPORTED INTERFACE)
set_target_properties(OSG::osg PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${OSG_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "${OSG_LIBRARY};${OSGDB_LIBRARY};${OSGGA_LIBRARY};${OSGUTIL_LIBRARY};${OSGVIEWER_LIBRARY};${OSGTEXT_LIBRARY};${OPENTHREADS_LIBRARY}"
)

# ============================================================================
# Eigen
# ============================================================================
set(EIGEN_ROOT "F:/eigen-3.4.1" CACHE PATH "Eigen root")
find_path(EIGEN_INCLUDE_DIR Eigen/Dense PATHS "${EIGEN_ROOT}")
if(NOT EIGEN_INCLUDE_DIR)
    message(FATAL_ERROR "Eigen not found at ${EIGEN_ROOT}")
endif()
add_library(Eigen::Eigen IMPORTED INTERFACE)
set_target_properties(Eigen::Eigen PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${EIGEN_INCLUDE_DIR}"
)

# ============================================================================
# 版本头
# ============================================================================
configure_file(
    ${CMAKE_SOURCE_DIR}/cmake/Version.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/include/Version.h
    @ONLY
)

# ============================================================================
# 子目录 — 按层构建
# ============================================================================
# OpenCV Debug 构建 ABI：同时定义 _DEBUG 和 DEBUG 确保 debug_build_guard 生效
if(MSVC)
    add_compile_definitions($<$<CONFIG:Debug>:DEBUG>)
endif()

add_subdirectory(framework)

# ============================================================================
# 模块
# ============================================================================
add_subdirectory(modules)

# ============================================================================
# SDK
# ============================================================================
add_subdirectory(sdk)

# ============================================================================
# 应用入口
# ============================================================================
add_subdirectory(app)
