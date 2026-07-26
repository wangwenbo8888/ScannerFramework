add_library(fw_data INTERFACE)
target_link_libraries(fw_data INTERFACE fw_common)

if(BUILD_TESTS)
    add_subdirectory(tests)
endif()
