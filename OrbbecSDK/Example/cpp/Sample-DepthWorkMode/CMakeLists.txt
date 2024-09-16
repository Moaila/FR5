#minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(Sample-DepthWorkMode)

add_executable(DepthWorkMode DepthWorkMode.cpp)

set_property(TARGET DepthWorkMode PROPERTY CXX_STANDARD 11)
target_link_libraries(DepthWorkMode OrbbecSDK ${OpenCV_LIBS})

