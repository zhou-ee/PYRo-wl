set(BOARD "CHASSIS_BOARD")

add_library(Infantry1 OBJECT)

target_compile_definitions(Infantry1 PRIVATE
        BOARD=${BOARD}
        CHASSIS_BOARD=2
)

target_compile_definitions(PYRo PUBLIC
    DR16_UART=PYRO_UART5
)

target_sources(Infantry1
        PRIVATE
        pyro_init_thread.cpp
        pyro_mission_planer.cpp

        Chassis/pyro_wl_chassis.cpp
        Chassis/fsm/pyro_wl_active_state.cpp
        Chassis/fsm/pyro_wl_manual_state.cpp
        Chassis/fsm/pyro_wl_passive_state.cpp
        Chassis/fsm/pyro_wl_normal_state.cpp
        Chassis/fsm/pyro_wl_align_state.cpp
        Chassis/fsm/pyro_wl_step_state.cpp


        Application/Chassis_board/pyro_wl_chassis_app.cpp
)

target_include_directories(Infantry1
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/Application/Chassis_board
        ${CMAKE_CURRENT_SOURCE_DIR}/Chassis
        ${CMAKE_CURRENT_SOURCE_DIR}/Communication/General
)

target_link_libraries(Infantry1
        PUBLIC
        stm32cubemx
        dsppp_lib
        PYRo
)
