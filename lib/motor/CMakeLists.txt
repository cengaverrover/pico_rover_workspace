add_library(pico_motor INTERFACE)

target_sources(pico_motor INTERFACE 
    motor.cpp 
    motor.hpp
)

target_link_libraries(pico_motor 
    INTERFACE 
        hardware_gpio
        hardware_pwm
        hardware_clocks
)

target_include_directories(pico_motor
    INTERFACE 
        ${CMAKE_CURRENT_SOURCE_DIR}  
)   