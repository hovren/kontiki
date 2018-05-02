cmake_minimum_required(VERSION 3.8)

project(Kontiki VERSION 0.9)

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

# Header-only library -> INTERFACE
add_library(Kontiki INTERFACE)

file(GLOB_RECURSE Kontiki_HEADER_FILES include/*.h)

# Associate target with include directory
target_include_directories(Kontiki INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

# Link / include dirs for dependencies
target_link_libraries(Kontiki INTERFACE Sophus::Sophus)
target_include_directories(Kontiki INTERFACE ${CERES_INCLUDE_DIRS})
target_link_libraries(Kontiki INTERFACE ${CERES_LIBRARIES})

include(GNUInstallDirs)
set(Kontiki_CMAKE_EXPORT_DIR ${CMAKE_INSTALL_DATADIR}/Kontiki/cmake)

# Not needed since the name is the same?
#set_target_properties(Kontiki PROPERTIES EXPORT_NAME Kontiki)

install(TARGETS Kontiki EXPORT KontikiTargets)
install(EXPORT KontikiTargets
    NAMESPACE Kontiki::
    DESTINATION ${Kontiki_CMAKE_EXPORT_DIR}
    )

export(TARGETS Kontiki NAMESPACE Kontiki:: FILE KontikiTargets.cmake)
export(PACKAGE Kontiki)

configure_package_config_file(
    KontikiConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/KontikiConfig.cmake
    INSTALL_DESTINATION ${Kontiki_CMAKE_EXPORT_DIR}
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

write_basic_package_version_file (
    KontikiConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

# Install cmake targets
install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/KontikiConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/KontikiConfigVersion.cmake
    DESTINATION ${Kontiki_CMAKE_EXPORT_DIR}
)

# Install header files
install(
    FILES ${Kontiki_HEADER_FILES}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/kontiki
)

add_custom_target(kontiki_lib_ide_target SOURCES ${Kontiki_HEADER_FILES})
