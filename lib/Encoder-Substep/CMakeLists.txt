
add_library(pico_encoder_substep INTERFACE)

pico_generate_pio_header(pico_encoder_substep ${CMAKE_CURRENT_LIST_DIR}/encoder_substep.pio)

target_sources(pico_encoder_substep INTERFACE 
    encoder_substep.h 
    encoder_substep.c
    encoder_substep.hpp
    encoder_substep_wrapper.cpp
)

target_include_directories(pico_encoder_substep 
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(pico_encoder_substep INTERFACE
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_pwm
    hardware_watchdog
)


add_library(pico_encoder_substep_class INTERFACE)

pico_generate_pio_header(pico_encoder_substep_class ${CMAKE_CURRENT_LIST_DIR}/encoder_substep.pio)

target_sources(pico_encoder_substep_class INTERFACE 
    encoder_substep.hpp
    encoder_substep_wrapper.cpp
)

target_include_directories(pico_encoder_substep_class 
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(pico_encoder_substep_class INTERFACE
    pico_encoder_substep
)