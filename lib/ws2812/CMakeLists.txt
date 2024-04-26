add_library(pico-ws2812 INTERFACE) 

pico_generate_pio_header(pico-ws2812 ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)

target_sources(pico-ws2812 
    INTERFACE
        ws2812.cpp
        ws2812.hpp
)

target_include_directories(pico-ws2812
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(pico-ws2812
    INTERFACE
        pico_stdlib
        hardware_pio
        hardware_clocks
)   