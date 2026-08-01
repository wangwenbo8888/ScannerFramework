# Module 06_device_control — 相机/MCU/硬件监控

set(GALAXY_SDK_DIR "${CMAKE_SOURCE_DIR}/sdk/third_party/galaxy_sdk")

add_library(mod_device_control STATIC
    CameraControl.cpp
    MCUDriver.cpp
    HardwareMonitor.cpp
)

target_include_directories(mod_device_control
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${GALAXY_SDK_DIR}/inc
        ${CMAKE_SOURCE_DIR}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(mod_device_control
    PUBLIC
        fw_hal
        fw_data
        fw_infra
        fw_common
        ${OpenCV_LIBS}
        spdlog::spdlog
    PRIVATE
        "${GALAXY_SDK_DIR}/lib/x64/GxIAPICPPEx.lib"
)

# Galaxy SDK DLL 运行时路径（构建后复制）
set(GALAXY_DLL_DIR "${GALAXY_SDK_DIR}/lib/x64" CACHE PATH "Galaxy DLL dir")
