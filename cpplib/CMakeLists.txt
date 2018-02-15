cmake_minimum_required(VERSION 3.8)

project(Taser VERSION 0.1.0)

include(CMakePackageConfigHelpers)

# Release by default
# Turn on Debug with "-DCMAKE_BUILD_TYPE=Debug"
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 14)

include(CMakeFindDependencyMacro)
find_dependency(Ceres REQUIRED)
find_dependency(Sophus)

find_package(Eigen3 REQUIRED)

# Header-only library -> INTERFACE
add_library(Taser INTERFACE)

if(TARGET Eigen3::Eigen)
    message("Eigen: First option")
    target_link_libraries(Taser INTERFACE Eigen3::Eigen)
    set(Eigen3_DEPENDENCY "find_dependency (Eigen3 ${Eigen3_VERSION})")
else(TARGET Eigen3::Eigen)
    message("Eigen: second option")
    target_include_directories (Taser SYSTEM INTERFACE ${EIGEN3_INCLUDE_DIR})
endif(TARGET Eigen3::Eigen)

#if (Sophus_FOUND)
#    message("Has Sophus, building SE3 trajectory")
#    message("Sophus dir: ${Sophus_DIR}")
#    message("Sophus include dirs: ${Sophus_INCLUDE_DIRS}")
#    target_link_libraries(Taser INTERFACE Sophus)
#else()
#    message("Sophus not found, not building SE3 trajectory")
#endif(Sophus_FOUND)

if(TARGET Sophus::Sophus)
    target_link_libraries(Taser INTERFACE Sophus::Sophus)
else()
    message("No sophus")
endif()

file(GLOB_RECURSE Taser_HEADER_FILES include/*.h)
#target_sources(Taser INTERFACE ${Taser_HEADER_FILES})

# Associate target with include directory
target_include_directories(Taser INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_include_directories(Taser INTERFACE ${CERES_INCLUDE_DIRS})
target_link_libraries(Taser INTERFACE ${CERES_LIBRARIES})

include(GNUInstallDirs)
set(Taser_CMAKE_EXPORT_DIR ${CMAKE_INSTALL_DATADIR}/Taser/cmake)

# Not needed since the name is the same?
#set_target_properties(Taser PROPERTIES EXPORT_NAME Taser)

install(TARGETS Taser EXPORT TaserTargets)
install(EXPORT TaserTargets
    NAMESPACE Taser::
    DESTINATION ${Taser_CMAKE_EXPORT_DIR}
    )

export(TARGETS Taser NAMESPACE Taser:: FILE TaserTargets.cmake)
export(PACKAGE Taser)

configure_package_config_file(
    TaserConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/TaserConfig.cmake
    INSTALL_DESTINATION ${Taser_CMAKE_EXPORT_DIR}
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

write_basic_package_version_file (
    TaserConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

# Install cmake targets
install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/TaserConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/TaserConfigVersion.cmake
    DESTINATION ${Taser_CMAKE_EXPORT_DIR}
)

# Install header files
install(
    FILES ${Taser_HEADER_FILES}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/taser
)