# Pico Rover Controller 

## Cloning repo
Project uses submodules:
+ [FreeRTOS Kernel](https://github.com/FreeRTOS/FreeRTOS-Kernel)
+ [libmicroros](https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk)
+ [Bno055_Driver](https://github.com/alpertng02/pico-bno055.git)

Clone using the --recurse-submodules 

## To build
Set the environment variable PICO_SDK_PATH to your sdk location.
- mkdir build && cd build
- cmake ..
- cmake --build . --config Release --target pico-rover 

