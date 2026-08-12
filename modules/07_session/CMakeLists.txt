# Module 07_session — 会话管理（EventBus + 工作流框架 + 状态机）
add_library(mod_session STATIC
    EventBus.cpp
    StateMachine.cpp
    SessionService.cpp
)

target_include_directories(mod_session
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}
)

target_link_libraries(mod_session
    PUBLIC
        spdlog::spdlog
        ${OpenCV_LIBS}
)
