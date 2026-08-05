set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

set(APP_SOURCES
    main.cpp
    AppContext.cpp
    AppContext.h
    MainWindow.cpp
    MainWindow.h
    OSGWidget.cpp
    OSGWidget.h
    CalibDialog.cpp
    CalibDialog.h
    CalibDisplay.cpp
    CalibDisplay.h
    IntegrateTestDialog.cpp
    IntegrateTestDialog.h
    ScannerWindow.cpp
    ScannerWindow.h
    ScannerWindow.ui
    calib_workflow.cpp
    calib_workflow.h
    file_io.cpp
    file_io.h
)

set(APP_RESOURCES
    resources.qrc
)

add_executable(scan_demo ${APP_SOURCES} ${APP_RESOURCES})

target_include_directories(scan_demo PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/stubs
    ${CMAKE_SOURCE_DIR}
)

target_link_libraries(scan_demo PRIVATE
    fw_common
    fw_data
    fw_infra
    fw_workflow
    mod_device_control
    mod_operatorlib
    Qt5::Widgets
    Qt5::OpenGL
    Qt5::Svg
    Qt5::SerialPort
    OSG::osg
    Eigen::Eigen
    ${OpenCV_LIBS}
)

target_compile_definitions(scan_demo PRIVATE
    NOMINMAX
    _CRT_SECURE_NO_WARNINGS
)

# ============================================================================
# Copy DLLs to output directory via batch script (handles Debug/Release)
# ============================================================================
add_custom_command(TARGET scan_demo POST_BUILD
    COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/copy_dlls.bat" "$<TARGET_FILE_DIR:scan_demo>" "$<CONFIG>"
    COMMENT "Copying DLLs to output directory..."
)
