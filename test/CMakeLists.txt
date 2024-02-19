cmake_minimum_required(VERSION 3.10)
project(test_demo)

include_directories(../util)
include_directories(../kdtree)
add_executable(test test.cpp ../util/utilities.cpp)


