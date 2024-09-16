#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(Sample-QuickStart)

add_executable(QuickStart QuickStart.cpp)

set_property(TARGET QuickStart PROPERTY CXX_STANDARD 11)
target_link_libraries(QuickStart OrbbecSDK )

