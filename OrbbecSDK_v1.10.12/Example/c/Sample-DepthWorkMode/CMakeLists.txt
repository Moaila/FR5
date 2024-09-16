cmake_minimum_required(VERSION 3.1.15)

set(CMAKE_CXX_STANDARD 11)
project(Sample-DepthWorkMode)
add_executable( depth_work_mode depth_work_mode.cpp)
target_link_libraries(depth_work_mode OrbbecSDK ${OpenCV_LIBS})