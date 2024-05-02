add_executable(${NAME}
        main.cpp
)

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(${NAME} 
	 pico_stdlib
     FreeRTOS-Kernel-Heap4
     micro_ros
     pico_micro_ros_transport
     pico_motor
     pico_encoder_substep
     pico_bno055
     pico-ws2812
)
	
target_include_directories(${NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../lib/FreeRTOS-Kernel/include
)
     
target_compile_definitions(${NAME} PRIVATE
    FREE_RTOS_KERNEL_SMP=1
    portSUPPORT_SMP=1
)

# Configure pin connections
target_compile_definitions(${NAME} PRIVATE 
    FRONT_LEFT_LPWM=2
    PRIVATE FRONT_LEFT_RPWM=3
    PRIVATE FRONT_LEFT_ENCODER_A=10

    PRIVATE FRONT_RIGHT_LPWM=4
    PRIVATE FRONT_RIGHT_RPWM=5
    PRIVATE FRONT_RIGHT_ENCODER_A=12

    PRIVATE BACK_LEFT_LPWM=6
    PRIVATE BACK_LEFT_RPWM=7
    PRIVATE BACK_LEFT_ENCODER_A=14

    PRIVATE BACK_RIGHT_LPWM=8
    PRIVATE BACK_RIGHT_RPWM=9
    PRIVATE BACK_RIGHT_ENCODER_A=20

    PRIVATE I2C_SDA_PIN=0
    PRIVATE I2C_SCL_PIN=1

    PRIVATE LED_STRIP_PIN=19
)

# create map/bin/hex file etc.
pico_add_extra_outputs(${NAME})

# enable usb output, disable uart output
pico_enable_stdio_usb(${NAME} 1)
pico_enable_stdio_uart(${NAME} 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)
#add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
#add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)
