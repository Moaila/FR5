cmake_minimum_required(VERSION 3.1.15)

set(CMAKE_CXX_STANDARD 11)
project(Sample-InfraredViewer)
add_executable( infrared_viewer infrared_viewer.cpp)
target_link_libraries(infrared_viewer OrbbecSDK ${OpenCV_LIBS})