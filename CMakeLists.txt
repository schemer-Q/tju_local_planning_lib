cmake_minimum_required(VERSION 3.20)

file(READ "${CMAKE_CURRENT_LIST_DIR}/config.json" config_json)

# get project version
string(JSON project_version GET ${config_json} project version)
string(JSON sdk_lidarnetsdk_version GET ${config_json} project dependencies sdk lidarnetsdk version)
string(JSON sdk_lidarnetsdk_enable GET ${config_json} project dependencies sdk lidarnetsdk enable)
string(JSON sdkbevlaneconedethighway_version GET ${config_json} project dependencies sdk sdkbevlaneconedethighway version)
string(JSON sdkbevlaneconedethighway_enable GET ${config_json} project dependencies sdk sdkbevlaneconedethighway enable)
string(JSON sdkvisdetr3ddconffront_version GET ${config_json} project dependencies sdk sdkvisdetr3ddconffront version)
string(JSON sdkvisdetr3ddconffront_enable GET ${config_json} project dependencies sdk sdkvisdetr3ddconffront enable)
string(JSON detectionnetsdk_version GET ${config_json} project dependencies sdk detectionnetsdk version)
string(JSON detectionnetsdk_enable GET ${config_json} project dependencies sdk detectionnetsdk enable)

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
# 检测系统版本
if(EXISTS "/etc/os-release")
    file(READ "/etc/os-release" OS_RELEASE_CONTENT)
    string(REGEX MATCH "VERSION_CODENAME=([a-z]*)" _ ${OS_RELEASE_CONTENT})
    set(OS_CODENAME ${CMAKE_MATCH_1})
    message(STATUS "OS_CODENAME: ${OS_CODENAME}")
endif()
string(TOLOWER ${PLATFORM} ${PLATFORM})
message(STATUS "Compile on platform: ${PLATFORM}")

# 获取GCC版本
message(STATUS "GCC C++ Compiler Version: ${CMAKE_CXX_COMPILER_VERSION}")
if (${CMAKE_CXX_COMPILER_VERSION} VERSION_LESS "9.0")
  add_definitions(-DOLD_GCC=1)
else()
  add_definitions(-DOLD_GCC=0)
endif()

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
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include(${CMAKE_CURRENT_LIST_DIR}/cmake/openmp.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/trunk_basic.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/trunk.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/google.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/pcl.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/opencv.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/eigen.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/yaml.cmake)



########################################################
# add subdirectory
########################################################
add_subdirectory(trunk_perception/common)
add_subdirectory(trunk_perception/tools/log)
add_subdirectory(trunk_perception/app)
add_subdirectory(trunk_perception/algo)

########################################################
# test
########################################################
if (ENABLE_TEST)
    message(STATUS "enable test")
    include_directories(${GTEST_INCLUDE_DIRS})
    message(STATUS "GTEST_INCLUDE_DIRS: ${GTEST_INCLUDE_DIRS}")
    message(STATUS "GTEST_LIBRARIES: ${GTEST_LIBRARIES}")
    enable_testing()
    add_subdirectory(trunk_perception/common/test)
endif()


########################################################
# install settings
########################################################

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/trunk_perception
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.hpp"
  PATTERN "*.h"
  PATTERN "*.yaml"
  )

configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/${TARGET_NAME}Config.cmake.in
        "${PROJECT_BINARY_DIR}/${TARGET_NAME}Config.cmake"
        INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${TARGET_NAME}"
        PATH_VARS CMAKE_INSTALL_INCLUDEDIR CMAKE_INSTALL_LIBDIR CMAKE_INSTALL_PREFIX)

write_basic_package_version_file("${PROJECT_BINARY_DIR}/${TARGET_NAME}ConfigVersion.cmake"
        VERSION ${project_version}
        COMPATIBILITY AnyNewerVersion)

install(FILES ${PROJECT_BINARY_DIR}/${TARGET_NAME}Config.cmake
        ${PROJECT_BINARY_DIR}/${TARGET_NAME}ConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_DATADIR}/cmake/${TARGET_NAME})
