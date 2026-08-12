# Module 06_fileio — 文件管理与数据管理
add_library(mod_fileio STATIC
    file_io.cpp
    PointCloudBuffer.cpp
    ParameterManager.cpp
)

target_include_directories(mod_fileio
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}
)

target_link_libraries(mod_fileio
    PUBLIC
        spdlog::spdlog
        ${OpenCV_LIBS}
        OSG::osg
)
