cmake_minimum_required(VERSION 3.1.15)

set(CMAKE_CXX_STANDARD 11)
project(Sample-AlignFilterViewer)
add_executable( align_filter_viewer align_filter_viewer.cpp)
target_link_libraries(align_filter_viewer OrbbecSDK ${OpenCV_LIBS})