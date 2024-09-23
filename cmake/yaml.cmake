find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIR})
message(STATUS "YAML_CPP version: ${yaml-cpp_VERSION} ${YAML_CPP_INCLUDE_DIR}")