find_package(Eigen3 REQUIRED NO_MODULE)
message(STATUS "Eigen3 version: ${Eigen3_VERSION}")
add_compile_options(-DEIGEN_STACK_ALLOCATION_LIMIT=0)
include_directories(${EIGEN3_INCLUDE_DIR})