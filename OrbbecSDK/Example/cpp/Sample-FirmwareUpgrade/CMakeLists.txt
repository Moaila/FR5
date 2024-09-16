#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(Sample-FirmwareUpgrade)

add_executable(FirmwareUpgrade FirmwareUpgrade.cpp)

set_property(TARGET FirmwareUpgrade PROPERTY CXX_STANDARD 11)
target_link_libraries(FirmwareUpgrade OrbbecSDK)
