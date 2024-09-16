cmake_minimum_required(VERSION 3.1.15)

set(CMAKE_CXX_STANDARD 11)
project(Sample-HdrMerge)
add_executable( hdr_merge hdr_merge.cpp)
target_link_libraries(hdr_merge OrbbecSDK ${OpenCV_LIBS})