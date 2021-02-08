cmake_minimum_required(VERSION 3.17)
set(PICO_SDK_PATH "C:\\pico\\pico-sdk")

include(pico_sdk_import.cmake)

project(FBW_Control_Pico)

pico_sdk_init()

set(CMAKE_CXX_STANDARD 17)

include_directories(src)
add_subdirectory(src)

include_directories(libs)
add_subdirectory(libs)

include_directories(Stream_data)
add_subdirectory(Stream_data)

