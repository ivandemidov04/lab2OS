cmake_minimum_required(VERSION 3.28)
project(lab2OS)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")

# Dynamic library
add_library(lab2_cache SHARED lab2_cache/lab2_cache.cpp)

add_executable(writebench writebench.cpp)
target_link_libraries(writebench lab2_cache)
target_include_directories(writebench PRIVATE lab2_cache)

add_executable(emasearchbench emasearchbench.cpp)
target_link_libraries(emasearchbench lab2_cache)
target_include_directories(emasearchbench PRIVATE lab2_cache)