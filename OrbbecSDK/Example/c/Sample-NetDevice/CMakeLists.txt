cmake_minimum_required(VERSION 3.1.15)

set(CMAKE_CXX_STANDARD 11)
project(Sample-NetDevice)
add_executable( net_device net_device.cpp)
target_link_libraries(net_device OrbbecSDK ${OpenCV_LIBS})