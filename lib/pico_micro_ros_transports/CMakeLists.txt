add_library(pico_micro_ros_uart_transport INTERFACE)

target_sources(pico_micro_ros_uart_transport INTERFACE
    pico_uart_transport.c
    pico_uart_transports.h
)

target_include_directories(pico_micro_ros_uart_transport INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(pico_micro_ros_uart_transport INTERFACE
    pico_stdlib
    microros
)

#------------------------------------------------------------

add_library(pico_micro_ros_usb_transport INTERFACE)

target_sources(pico_micro_ros_usb_transport INTERFACE
    pico_usb_transports.c
    pico_usb_transports.h
)

target_include_directories(pico_micro_ros_usb_transport INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(pico_micro_ros_usb_transport INTERFACE
    pico_stdlib
    pico_stdio
    microros
)