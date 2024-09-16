#  minimum required cmake version: 3.1.15 support vs2019

set(CMAKE_CXX_STANDARD 11)
project(Sample-PointCloud)
add_executable( point_cloud point_cloud.c)
target_link_libraries(point_cloud OrbbecSDK ${OpenCV_LIBS})