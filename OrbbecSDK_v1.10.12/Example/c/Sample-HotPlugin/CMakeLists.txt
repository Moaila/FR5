#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(Sample-HotPlugin)

set(CMAKE_CXX_STANDARD 11)

add_executable(hot_plugin hot_plugin.cpp)

target_link_libraries(hot_plugin OrbbecSDK   )

