#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(Sample-DoubleInfraredViewer)

add_executable(DoubleInfraredViewer DoubleInfraredViewer.cpp)

set_property(TARGET DoubleInfraredViewer PROPERTY CXX_STANDARD 11)
target_link_libraries(DoubleInfraredViewer OrbbecSDK ${OpenCV_LIBS})
