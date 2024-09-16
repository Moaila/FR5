#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(Sample-SyncAlignViewer)

add_executable(SyncAlignViewer SyncAlignViewer.cpp)

set_property(TARGET SyncAlignViewer PROPERTY CXX_STANDARD 11)
target_link_libraries(SyncAlignViewer OrbbecSDK ${OpenCV_LIBS})