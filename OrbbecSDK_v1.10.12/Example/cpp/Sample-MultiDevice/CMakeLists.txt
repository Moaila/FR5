#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(Sample-MultiDevice)

add_executable(MultiDevice MultiDevice.cpp)

set_property(TARGET MultiDevice PROPERTY CXX_STANDARD 11)
target_link_libraries(MultiDevice OrbbecSDK ${OpenCV_LIBS})

