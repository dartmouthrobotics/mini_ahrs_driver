cmake_minimum_required(VERSION 3.0)
PROJECT(MiniAHRSDriver VERSION 0.0.0 LANGUAGES CXX)

set(THREADS_PREFER_PTHREAD_FLAG ON)

FILE(GLOB all_source_files src/*.cpp)

add_library(mini_ahrs_driver ${all_source_files})

add_executable(stag_main src/main.cpp)
target_link_libraries(stag_main ${OpenCV_LIBS} stag_lib)
