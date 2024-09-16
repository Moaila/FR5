cmake_minimum_required(VERSION 3.1.15)
set(CMAKE_CXX_STANDARD 11)
project(Sample-MultiDeviceSync)
add_executable( MultiDeviceSync
    MultiDeviceSync.cpp
    cJSON.c
)

target_link_libraries(MultiDeviceSync OrbbecSDK ${OpenCV_LIBS})