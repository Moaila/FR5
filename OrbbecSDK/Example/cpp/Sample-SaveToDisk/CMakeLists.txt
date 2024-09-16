#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(Sample-SaveToDisk)

add_executable(SaveToDisk SaveToDisk.cpp)

set_property(TARGET SaveToDisk PROPERTY CXX_STANDARD 11)
target_link_libraries(SaveToDisk OrbbecSDK ${OpenCV_LIBS} )

