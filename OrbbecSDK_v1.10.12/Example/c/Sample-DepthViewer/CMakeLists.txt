cmake_minimum_required(VERSION 3.1.15)

set(CMAKE_CXX_STANDARD 11)
project(Sample-DepthViewer)
add_executable( depth_viewer depth_viewer.cpp)
target_link_libraries(depth_viewer OrbbecSDK ${OpenCV_LIBS})