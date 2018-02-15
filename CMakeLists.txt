cmake_minimum_required(VERSION 3.8)
project(taserv2)

set(CMAKE_CXX_STANDARD 14)

#set(CMAKE_VERBOSE_MAKEFILE 1)

message("--- C++ library")
add_subdirectory(cpplib)

message("--- Python bindings")
add_subdirectory(python)