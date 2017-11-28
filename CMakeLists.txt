cmake_minimum_required(VERSION 3.8)
project(taserv2)

# FIXME: Necessary on my machine to find Eigen
set(CMAKE_MODULE_PATH /usr/share/cmake/Modules)

set(CMAKE_CXX_STANDARD 14)

# Eigen
find_package(Eigen3 REQUIRED)
message("Eigen dir: ${EIGEN3_INCLUDE_DIR}")

# Ceres
find_package(Ceres REQUIRED)
message("Ceres inc dir: ${CERES_INCLUDE_DIRS}")

# List include directories
include_directories(include
        ${EIGEN3_INCLUDE_DIR}
        ${CERES_INCLUDE_DIRS})

# Generate python module
add_subdirectory(deps/pybind11)
pybind11_add_module(dummy src/dummy.cpp)
