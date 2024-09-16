cmake_minimum_required(VERSION 3.1.15)

set(CMAKE_CXX_STANDARD 11)
project(Sample-ColorViewer)
add_executable( color_viewer color_viewer.cpp)
target_link_libraries(color_viewer OrbbecSDK ${OpenCV_LIBS})