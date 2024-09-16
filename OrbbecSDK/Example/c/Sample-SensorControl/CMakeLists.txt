#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(Sample-SensorControl)

add_executable(sensor_control sensor_control.c)

set_property(TARGET sensor_control PROPERTY C_STANDARD 11)

target_link_libraries(sensor_control OrbbecSDK   )