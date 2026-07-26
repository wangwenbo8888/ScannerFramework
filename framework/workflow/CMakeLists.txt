add_library(fw_workflow STATIC Pipeline.cpp)
target_include_directories(fw_workflow PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(fw_workflow PUBLIC fw_common fw_data)

if(BUILD_TESTS)
    add_subdirectory(tests)
endif()
