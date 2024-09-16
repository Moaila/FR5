#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(Sample-HelloOrbbec)

add_executable(HelloOrbbec HelloOrbbec.cpp)

set_property(TARGET HelloOrbbec PROPERTY CXX_STANDARD 11)
target_link_libraries(HelloOrbbec OrbbecSDK   )

