cmake_minimum_required(VERSION 3.13)

# Change your executable name to something creative!
set(NAME pico-rover) # <-- Name your project/executable here!

include(pico_sdk_import.cmake)

# Gooey boilerplate
project(${NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

#include Libraries
SET(FREERTOS_CONFIG_FILE_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/port/FreeRTOS-Kernel" CACHE STRING "Local Config")
include_directories("${FREERTOS_CONFIG_FILE_DIRECTORY}") 
include(FreeRTOS_Kernel_import.cmake)
add_compile_definitions(
    FREE_RTOS_KERNEL_SMP=1
    portSUPPORT_SMP=1
)

SET(MICRO_ROS_PATH "${CMAKE_CURRENT_LIST_DIR}/lib/micro_ros_raspberrypi_pico_sdk/" CACHE STRING "Course Common Lib")
include(micro_ros.cmake)


add_subdirectory(lib/pico_micro_ros_transport)
add_subdirectory(lib/Encoder-Substep)
add_subdirectory(lib/motor)
add_subdirectory(lib/bno055_lib/lib)

#Add main source directory
add_subdirectory(src)