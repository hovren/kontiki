cmake_minimum_required(VERSION 3.8)

project(kontiki-python)

set(CMAKE_CXX_STANDARD 14)

list(APPEND SEARCH_HEADERS ${EIGEN3_INCLUDE_DIR})

# The following is only used for debug prints
find_package(PythonInterp)
message("--- PYTHON configuration")
message("PYTHON_EXECUTABLE: ${PYTHON_EXECUTABLE}")
message("PYTHON_VERSION_STRING: ${PYTHON_VERSION_STRING}")

#find_package(Kontiki REQUIRED)
message("Kontiki found: ${KONTIKI_FOUND}")
message("Kontiki: ${Kontiki_INCLUDE_DIRS}")

# Load pybind11
add_subdirectory(deps/pybind11)

#add_subdirectory(deps/hana)

# Build a Python extension module using pybind11
#   kontiki_add_module(<module>)
# Here <module> should be the fully qualifued name for the module,
# e.g. kontiki.trajectories._dummy_trajectory
# <module> becomes the target name in case you wish to do something to it later
function(kontiki_add_module module)
    set(target_name ${module})
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

    #message("module='${module}' modname='${modname}' relpath='${relpath}'")

    # Define the binding source file
    set(sources src/${relpath}/py${modname}.cc)

    # Invoke pybind11 and set where the library should go, and what it is called
    pybind11_add_module(${target_name} ${sources})
    set(outdir ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${relpath})
    set_target_properties(${target_name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${outdir})
    set_target_properties(${target_name} PROPERTIES OUTPUT_NAME ${modname})
    target_link_libraries(${target_name} PRIVATE Kontiki)
    target_include_directories(${target_name} PRIVATE deps/hana/include)
endfunction()

# Dummy module for testing
#kontiki_add_module(kontiki.dummy)

# Ceres types
kontiki_add_module(kontiki._ceres)

# Trajectories
kontiki_add_module(kontiki.trajectories._linear_trajectory)
kontiki_add_module(kontiki.trajectories._uniform_r3_spline_trajectory)
kontiki_add_module(kontiki.trajectories._uniform_so3_spline_trajectory)
kontiki_add_module(kontiki.trajectories._uniform_se3_spline_trajectory)
kontiki_add_module(kontiki.trajectories._split_trajectory)

# Sensors
# - Cameras
kontiki_add_module(kontiki.sensors._pinhole_camera)
kontiki_add_module(kontiki.sensors._atan_camera)

# - IMUs
kontiki_add_module(kontiki.sensors._basic_imu)
kontiki_add_module(kontiki.sensors._constant_bias_imu)

# Structire from motion
kontiki_add_module(kontiki.sfm)

# Trajectory estimator
kontiki_add_module(kontiki._trajectory_estimator)

# Measurements
kontiki_add_module(kontiki.measurements._position_measurement)
kontiki_add_module(kontiki.measurements._orientation_measurement)
kontiki_add_module(kontiki.measurements._accelerometer_measurement)
kontiki_add_module(kontiki.measurements._gyroscope_measurement)
kontiki_add_module(kontiki.measurements._static_rscamera_measurement)
kontiki_add_module(kontiki.measurements._lifting_rscamera_measurement)
kontiki_add_module(kontiki.measurements._newton_rscamera_measurement)

# IDE helper target
file(GLOB_RECURSE python_sources src/*.cc src/*.h)
add_custom_target(kontiki_python_ide_target SOURCES ${python_sources})