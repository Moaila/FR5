#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(Sample-HelloOrbbec)

add_executable(hello_orbbec hello_orbbec.c)

set_property(TARGET hello_orbbec PROPERTY C_STANDARD 11)

target_link_libraries(hello_orbbec OrbbecSDK   )