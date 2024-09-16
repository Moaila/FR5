#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(Sample-Playback)

add_executable(Playback Playback.cpp)

set_property(TARGET Playback PROPERTY CXX_STANDARD 11)
target_link_libraries(Playback OrbbecSDK ${OpenCV_LIBS})