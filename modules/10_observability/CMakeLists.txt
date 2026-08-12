# Module 10_observability — 可观测性/运维（故障处理）
add_library(mod_observability STATIC
    FaultHandler.cpp
)

target_include_directories(mod_observability
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}
)

target_link_libraries(mod_observability
    PUBLIC
        mod_session
        spdlog::spdlog
)
