add_library(fw_workflow STATIC
    Pipeline.cpp
    WorkflowContext.cpp
    ScanWorkflow.cpp
    CalibrationWorkflow.cpp
    PostProcessWorkflow.cpp
)
target_include_directories(fw_workflow PUBLIC ${CMAKE_SOURCE_DIR})
target_link_libraries(fw_workflow PUBLIC
    fw_common
    fw_data
    fw_service
    fw_infra
    fw_hal
    spdlog::spdlog
    ${OpenCV_LIBS}
)
target_link_libraries(fw_workflow PRIVATE mod_operatorlib)

if(BUILD_TESTS)
    add_subdirectory(tests)
endif()
