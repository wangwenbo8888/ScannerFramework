# mod_operatorlib: 算子库 (core17 + calib16 + scan6 + GBA)
# 来源: JEAMMWARE260705 迁移 (命名空间保留 calib::)
#
# BUILD_CUDA=ON  → 编译全部 (CPU+CUDA+可选GBA)
# BUILD_CUDA=OFF → 仅编译CPU算子 (过滤 .cu 和 _cuda.cpp)

# === 收集所有源 ===
file(GLOB_RECURSE OP_ALL_CPP
    "${CMAKE_CURRENT_SOURCE_DIR}/core/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/calibration/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/scanning/*.cpp")
file(GLOB_RECURSE OP_ALL_CU
    "${CMAKE_CURRENT_SOURCE_DIR}/core/*.cu"
    "${CMAKE_CURRENT_SOURCE_DIR}/calibration/*.cu"
    "${CMAKE_CURRENT_SOURCE_DIR}/scanning/*.cu")

set(OP_SOURCES "")
foreach(f ${OP_ALL_CPP} ${OP_ALL_CU})
    if(f MATCHES "/tests/|bench_|benchmark_|example_usage|gba_dataset_runner")
        continue()
    elseif(NOT BUILD_CUDA AND f MATCHES "_cuda\\.cpp$")
        continue()
    elseif(NOT BUILD_CUDA AND f MATCHES "\\.cu$")
        continue()
    elseif(NOT BUILD_GLOBAL_OPTIM AND f MATCHES "global_ba_cpu\\.cpp$")
        continue()
    else()
        list(APPEND OP_SOURCES ${f})
    endif()
endforeach()

# === 收集头文件目录 → include 路径 ===
file(GLOB_RECURSE OP_HEADERS
    "${CMAKE_CURRENT_SOURCE_DIR}/core/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/calibration/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/scanning/*.h")
set(OP_INC_DIRS "")
foreach(h ${OP_HEADERS})
    get_filename_component(d ${h} DIRECTORY)
    list(APPEND OP_INC_DIRS ${d})
endforeach()
list(REMOVE_DUPLICATES OP_INC_DIRS)

# === 构建库 ===
if(BUILD_CUDA)
    enable_language(CUDA)
    set(CMAKE_CUDA_STANDARD 17)
    set(CMAKE_CUDA_ARCHITECTURES 89)
    set(CMAKE_CUDA_DEPENDENCIES_USE_COMPILER OFF)
    string(APPEND CMAKE_CUDA_FLAGS " --extended-lambda -Xcompiler=/utf-8")
    # 手动覆盖 VS 生成的 CUDA AdditionalOptions（移除 nvcc 无法识别的 /MP /W4 /arch:SSE2）
    # 方案：编译后修补 vcxproj，下次构建生效
endif()

add_library(mod_operatorlib STATIC ${OP_SOURCES})

# VS CUDA 兼容：构建前修补 vcxproj（通过外部脚本在构建开始前执行）
if(MSVC AND BUILD_CUDA)
    # 创建修补脚本，在第一次 configure 后手动运行
    configure_file(
        "${CMAKE_SOURCE_DIR}/cmake/PatchCudaVcxproj.bat.in"
        "${CMAKE_CURRENT_BINARY_DIR}/patch_cuda.bat"
        @ONLY)
endif()

target_include_directories(mod_operatorlib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/core
    ${CMAKE_SOURCE_DIR}
    ${OP_INC_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(mod_operatorlib PUBLIC
    ${OpenCV_LIBS}
    Eigen::Eigen
    spdlog::spdlog
    nlohmann_json::nlohmann_json
)

if(BUILD_CUDA)
    target_compile_definitions(mod_operatorlib PUBLIC BUILD_CUDA=1)
else()
    target_compile_definitions(mod_operatorlib PUBLIC BUILD_CUDA=0)
endif()

target_compile_definitions(mod_operatorlib PRIVATE LM_ENABLE_TIMING=1)

if(MSVC)
    target_compile_options(mod_operatorlib PRIVATE
        $<$<COMPILE_LANGUAGE:CXX>:/bigobj>
        $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=/bigobj>)
endif()

# === Ceres (GBA) ===
if(BUILD_GLOBAL_OPTIM)
    find_package(Ceres QUIET)
    if(Ceres_FOUND)
        target_link_libraries(mod_operatorlib PUBLIC ${CERES_LIBRARIES})
        target_include_directories(mod_operatorlib PUBLIC ${CERES_INCLUDE_DIRS})
    else()
        message(STATUS "Ceres not found — GBA sources excluded")
    endif()
endif()
