add_library(pico_micro_ros_transport INTERFACE)

target_sources(pico_micro_ros_transport INTERFACE
    pico_uart_transport.c
    pico_uart_transports.h
)

target_include_directories(pico_micro_ros_transport INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(pico_micro_ros_transport INTERFACE
    pico_stdlib
    microros
)