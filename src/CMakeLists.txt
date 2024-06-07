add_executable(${NAME}
        main.cpp
)

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(${NAME} 
	 pico_stdlib
     pico_stdio
     FreeRTOS-Kernel-Heap4
     micro_ros
     pico_micro_ros_usb_transport
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

target_compile_definitions(${NAME} PRIVATE
    ON_BOTH_CORES=0x03
    ON_CORE_ZERO=0x01
    ON_CORE_ONE=0x02
)

target_compile_definitions(${NAME} PRIVATE
    PWM_MAX_DUTY_CYCLE=80.0f
    MOTOR_MAX_RPM=16000
    ENCODER_PULSE_PER_REV=16

    LED_STRIP_SIZE=119
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
pico_enable_stdio_uart(${NAME} 1)
add_compile_definitions(PICO_USB_ENABLE_CRLF_SUPPORT=0)

target_compile_definitions(${NAME} PRIVATE
    PICO_DEFAULT_UART_RX_PIN=16
    PICO_DEFAULT_UART_TX_PIN=17
)
