cmake_minimum_required(VERSION 3.12)

project(serial)

# aris use c++ 17 standard
set(CMAKE_CXX_STANDARD 17)

add_library(myfunctions STATIC serial.cpp)
add_executable(serial main.cpp serial.cpp serial.h)

target_link_libraries(serial myfunctions)


