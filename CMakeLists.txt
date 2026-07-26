cmake_minimum_required(VERSION 3.24)

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
option(ENABLE_CUDA "Enable CUDA support" OFF)
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
find_package(OpenCV 4.13 REQUIRED COMPONENTS core calib3d imgproc imgcodecs)

if(ENABLE_CUDA)
    find_package(CUDAToolkit 12.6 REQUIRED)
endif()

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
