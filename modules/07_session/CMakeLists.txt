# Module 07_session — 会话管理（EventBus 事件总线）
add_library(mod_session STATIC
    EventBus.cpp
)

target_include_directories(mod_session
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}
)

target_link_libraries(mod_session
    PUBLIC
        fw_common
        spdlog::spdlog
)
