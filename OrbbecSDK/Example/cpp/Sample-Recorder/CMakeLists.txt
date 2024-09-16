#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(Sample-Recorder)

add_executable(Recorder Recorder.cpp)

set_property(TARGET Recorder PROPERTY CXX_STANDARD 11)
target_link_libraries(Recorder OrbbecSDK ${OpenCV_LIBS})