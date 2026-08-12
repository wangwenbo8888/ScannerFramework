set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC OFF)

# === app 核心文件 ===
set(APP_SOURCES
    main.cpp
    AppContext.cpp
    AppContext.h
    MainWindow.cpp
    MainWindow.h
    Pipeline.cpp
    Pipeline.h
    WorkflowContext.cpp
    WorkflowContext.h
    IWorkflow.h
)

# === 模块文件（物理在 modules/ 下，编译进 scan_demo）===
set(MOD_CALIBRATION
    ${CMAKE_SOURCE_DIR}/modules/01_calibration/calib_workflow.cpp
    ${CMAKE_SOURCE_DIR}/modules/01_calibration/calib_workflow.h
    ${CMAKE_SOURCE_DIR}/modules/01_calibration/CalibDialog.cpp
    ${CMAKE_SOURCE_DIR}/modules/01_calibration/CalibDialog.h
    ${CMAKE_SOURCE_DIR}/modules/01_calibration/CalibDisplay.cpp
    ${CMAKE_SOURCE_DIR}/modules/01_calibration/CalibDisplay.h
    ${CMAKE_SOURCE_DIR}/modules/01_calibration/IntegrateTestDialog.cpp
    ${CMAKE_SOURCE_DIR}/modules/01_calibration/IntegrateTestDialog.h
    ${CMAKE_SOURCE_DIR}/modules/01_calibration/CalibrationWorkflow.cpp
    ${CMAKE_SOURCE_DIR}/modules/01_calibration/CalibrationWorkflow.h
    ${CMAKE_SOURCE_DIR}/modules/01_calibration/CalibStore.cpp
    ${CMAKE_SOURCE_DIR}/modules/01_calibration/CalibStore.h
)

set(MOD_SCANNING
    ${CMAKE_SOURCE_DIR}/modules/02_scanning/ScannerWindow.cpp
    ${CMAKE_SOURCE_DIR}/modules/02_scanning/ScannerWindow.h
    ${CMAKE_SOURCE_DIR}/modules/02_scanning/ScannerWindow.ui
    ${CMAKE_SOURCE_DIR}/modules/02_scanning/ScanWorkflow.cpp
    ${CMAKE_SOURCE_DIR}/modules/02_scanning/ScanWorkflow.h
    ${CMAKE_SOURCE_DIR}/modules/02_scanning/ScanConfig.h
    ${CMAKE_SOURCE_DIR}/modules/02_scanning/ScanPipeline.cpp
    ${CMAKE_SOURCE_DIR}/modules/02_scanning/ScanPipeline.h
)

set(MOD_POSTPROCESSING
    ${CMAKE_SOURCE_DIR}/modules/04_postprocessing/PostProcessWorkflow.cpp
    ${CMAKE_SOURCE_DIR}/modules/04_postprocessing/PostProcessWorkflow.h
)

set(MOD_RENDERING
    ${CMAKE_SOURCE_DIR}/modules/03_rendering/OSGWidget.cpp
    ${CMAKE_SOURCE_DIR}/modules/03_rendering/OSGWidget.h
)

set(MOD_FILEIO
    ${CMAKE_SOURCE_DIR}/modules/06_fileio/file_io.cpp
    ${CMAKE_SOURCE_DIR}/modules/06_fileio/file_io.h
)

set(GENERATED_QRC_CPP ${CMAKE_CURRENT_BINARY_DIR}/qrc_resources.cpp)
add_custom_command(
    OUTPUT ${GENERATED_QRC_CPP}
    COMMAND "C:/Qt/Qt5.15.2/5.15.2/msvc2019_64/bin/rcc.exe" -name resources
            -o "${GENERATED_QRC_CPP}" "${CMAKE_CURRENT_SOURCE_DIR}/resources.qrc"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/resources.qrc"
    COMMENT "Generating qrc_resources.cpp"
    VERBATIM
)

add_executable(scan_demo
    ${APP_SOURCES}
    ${MOD_CALIBRATION}
    ${MOD_SCANNING}
    ${MOD_POSTPROCESSING}
    ${MOD_RENDERING}
    ${MOD_FILEIO}
    ${GENERATED_QRC_CPP}
)

target_include_directories(scan_demo PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/stubs
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/modules/01_calibration
    ${CMAKE_SOURCE_DIR}/modules/02_scanning
    ${CMAKE_SOURCE_DIR}/modules/03_rendering
    ${CMAKE_SOURCE_DIR}/modules/06_fileio
    ${CMAKE_SOURCE_DIR}/modules/04_postprocessing
)

target_link_libraries(scan_demo PRIVATE
    mod_devicemgmt
    mod_session
    mod_fileio
    mod_observability
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
