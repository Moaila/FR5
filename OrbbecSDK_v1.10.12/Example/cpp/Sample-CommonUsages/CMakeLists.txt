#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(Sample-CommonUsages)

add_executable(CommonUsages CommonUsages.cpp)

set_property(TARGET CommonUsages PROPERTY CXX_STANDARD 11)
target_link_libraries(CommonUsages OrbbecSDK ${OpenCV_LIBS})

