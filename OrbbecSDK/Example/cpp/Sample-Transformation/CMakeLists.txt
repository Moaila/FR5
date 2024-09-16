#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(Sample-Transformation)

add_executable(Transformation Transformation.cpp)

set_property(TARGET Transformation PROPERTY CXX_STANDARD 11)
target_link_libraries(Transformation OrbbecSDK ${OpenCV_LIBS})