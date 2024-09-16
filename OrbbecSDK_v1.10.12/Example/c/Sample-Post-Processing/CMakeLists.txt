cmake_minimum_required(VERSION 3.1.15)

set(CMAKE_CXX_STANDARD 11)
project(Sample-Post-Processing)
add_executable( post_processing post_processing.cpp)
target_link_libraries(post_processing OrbbecSDK ${OpenCV_LIBS})