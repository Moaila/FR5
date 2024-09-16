#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(Sample-MultiStream)

add_executable(MultiStream MultiStream.cpp)

set_property(TARGET MultiStream PROPERTY CXX_STANDARD 11)
target_link_libraries(MultiStream OrbbecSDK ${OpenCV_LIBS})