cmake_minimum_required(VERSION 3.1.15)

set(CMAKE_CXX_STANDARD 11)
project(Sample-DoubleInfraredViewer)
add_executable( double_infrared_viewer double_infrared_viewer.cpp)
target_link_libraries(double_infrared_viewer OrbbecSDK ${OpenCV_LIBS})