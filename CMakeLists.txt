cmake_minimum_required(VERSION 3.8)
project(taserv2)

# FIXME: Necessary on my machine to find Eigen
set(CMAKE_MODULE_PATH /usr/share/cmake/Modules)

set(CMAKE_CXX_STANDARD 14)

# The following is only used for debug prints
find_package(PythonInterp)
message("--- PYTHON configuration")
message("PYTHON_EXECUTABLE: ${PYTHON_EXECUTABLE}")
message("PYTHON_VERSION_STRING: ${PYTHON_VERSION_STRING}")

# Eigen
find_package(Eigen3 REQUIRED)
message("Eigen dir: ${EIGEN3_INCLUDE_DIR}")

# Ceres
find_package(Ceres REQUIRED)
message("Ceres inc dir: ${CERES_INCLUDE_DIRS}")

# List include directories
include_directories(
    include
    include/taser
    deps/hana/include
    ${EIGEN3_INCLUDE_DIR}
    ${CERES_INCLUDE_DIRS})

# Load pybind11
add_subdirectory(deps/pybind11)

# Build a Python extension module using pybind11
#   taser_add_module(<module>)
# Here <module> should be the fully qualifued name for the module,
# e.g. taser.trajectories._dummy_trajectory
# <module> becomes the target name in case you wish to do something to it later
function(taser_add_module module)
    set(target_name ${module})
    string(REPLACE "." "/" modpath ${module})
    string(REPLACE "." ";" modlist ${module})

    # The module name is the last entry
    list(GET modlist -1 modname)

    # Remove everything that is not the root or the module name
    #list(REMOVE_AT modlist 0)
    list(REMOVE_AT modlist -1)

    # Get the relative path
    if(modlist)
        string(REPLACE ";" "/" relpath "${modlist}")
    else()
        set(relpath "")
    endif()

    # Define the binding source file
    set(sources src/pybindings/${relpath}/py${modname}.cc)

    # Invoke pybind11 and set where the library should go, and what it is called
    pybind11_add_module(${target_name} ${sources})
    set(outdir ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${relpath})
    set_target_properties(${target_name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${outdir})
    set_target_properties(${target_name} PROPERTIES OUTPUT_NAME ${modname})
    target_link_libraries(${target_name} PRIVATE ${CERES_LIBRARIES})
endfunction()

# Dummy module for testing
#taser_add_module(taser.dummy)

# Ceres types
taser_add_module(taser._ceres)

# Trajectories
taser_add_module(taser.trajectories._linear_trajectory)
taser_add_module(taser.trajectories._uniform_r3_spline_trajectory)
taser_add_module(taser.trajectories._uniform_so3_spline_trajectory)
taser_add_module(taser.trajectories._split_trajectory)

# Cameras
taser_add_module(taser.cameras._pinhole_camera)
taser_add_module(taser.cameras._atan_camera)

# IMUs
taser_add_module(taser.sensors._basic_imu)
taser_add_module(taser.sensors._constant_bias_imu)

# Structire from motion
taser_add_module(taser.sfm)

# Trajectory estimator
taser_add_module(taser._trajectory_estimator)

# Measurements
taser_add_module(taser.measurements._position_measurement)
taser_add_module(taser.measurements._gyroscope_measurement)
taser_add_module(taser.measurements._static_rscamera_measurement)

add_executable(entity_devel src/entity_devel.cc)
target_link_libraries(entity_devel PRIVATE ${CERES_LIBRARIES})


# CLion requires projects to use CMake, and this tells it about our headers
# Please reload the project within CLion when adding new sources
file(GLOB_RECURSE our_lib_headers include/*.h)
file(GLOB_RECURSE our_srcs src/*.*)
add_custom_target(ide_clion_dummy SOURCES ${our_lib_headers} ${our_srcs})