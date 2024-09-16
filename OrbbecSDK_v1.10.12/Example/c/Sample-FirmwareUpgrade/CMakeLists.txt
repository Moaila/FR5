#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(Sample-FirmwareUpgrade)

set(CMAKE_CXX_STANDARD 11)

add_executable(firmware_upgrade firmware_upgrade.cpp)

target_link_libraries(firmware_upgrade OrbbecSDK)
