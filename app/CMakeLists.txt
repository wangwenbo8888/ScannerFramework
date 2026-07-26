add_executable(scan_demo main.cpp)
target_link_libraries(scan_demo PRIVATE fw_common fw_infra fw_workflow jmw_sdk)
