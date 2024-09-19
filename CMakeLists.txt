cmake_minimum_required(VERSION 3.20)

file(READ "${CMAKE_CURRENT_LIST_DIR}/config.json" config_json)

# get project version
string(JSON project_version GET ${config_json} project version)

project(trunk_perception
        VERSION ${project_version}
        DESCRIPTION "trunk perception library"
        LANGUAGES CXX CUDA)
set(TARGET_NAME ${PROJECT_NAME})

include(${CMAKE_CURRENT_LIST_DIR}/cmake/cpack.cmake)
include(CMakePackageConfigHelpers)
include(CMakeDependentOption)
include(CheckCCompilerFlag)
include(GNUInstallDirs)
if (NOT PACKAGE_HUB)
  trunk_make_package(${TARGET_NAME} ${PROJECT_VERSION} ${PROJECT_DESCRIPTION})
endif ()


########################################################
# Platform
########################################################
if (DEFINED PLATFORM AND NOT ${PLATFORM} STREQUAL "")
  set(PLATFORM ${PLATFORM})
else ()
  set(PLATFORM "x86")
endif()
string(TOLOWER ${PLATFORM} ${PLATFORM})
message(STATUS "Compile on platform: ${PLATFORM}")


########################################################
# Global Settings
########################################################
set(CMAKE_CXX_STANDARD 17)
if (NOT DEFINED CMAKE_BUILD_TYPE OR "${CMAKE_BUILD_TYPE}" STREQUAL "")
    set(CMAKE_BUILD_TYPE "Release")
else()
    set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE})
endif()
message(STATUS "build type: ${CMAKE_BUILD_TYPE}")
if (CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
elseif (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -O1")
else()
    message(FATAL_ERROR "Invalid build type: ${CMAKE_BUILD_TYPE}")
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
option(ENABLE_TEST "enable test" ON)



########################################################
# Dependency
########################################################
include(${CMAKE_CURRENT_LIST_DIR}/cmake/trunk_basic.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/google.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/pcl.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/opencv.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/eigen.cmake)



########################################################
# add subdirectory
########################################################
include_directories(${CMAKE_CURRENT_LIST_DIR}/perception)

add_subdirectory(perception/common)
add_subdirectory(perception/tools/log)
# add_subdirectory(algo)
# add_subdirectory(deprecated_algo)
add_subdirectory(perception/app)

########################################################
# test
########################################################
if (ENABLE_TEST)
    message(STATUS "enable test")
    include_directories(${GTEST_INCLUDE_DIRS})
    message(STATUS "GTEST_INCLUDE_DIRS: ${GTEST_INCLUDE_DIRS}")
    message(STATUS "GTEST_LIBRARIES: ${GTEST_LIBRARIES}")
    enable_testing()
    add_subdirectory(perception/common/test)
endif()


########################################################
# install settings
########################################################
configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/${TARGET_NAME}Config.cmake.in
        "${PROJECT_BINARY_DIR}/${TARGET_NAME}Config.cmake"
        INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${TARGET_NAME}"
        PATH_VARS CMAKE_INSTALL_INCLUDEDIR CMAKE_INSTALL_LIBDIR CMAKE_INSTALL_PREFIX)

write_basic_package_version_file("${PROJECT_BINARY_DIR}/${TARGET_NAME}ConfigVersion.cmake"
        VERSION ${project_version}
        COMPATIBILITY AnyNewerVersion)

install(FILES ${PROJECT_BINARY_DIR}/${TARGET_NAME}Config.cmake
        ${PROJECT_BINARY_DIR}/${TARGET_NAME}ConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${TARGET_NAME})
