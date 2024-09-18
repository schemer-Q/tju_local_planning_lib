# boost
find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})

# pcl
set(FPHSA_NAME_MISMATCHED 1)
find_package(PCL REQUIRED)
message(STATUS "PCL version: ${PCL_VERSION}")
message(STATUS "PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")
include_directories(${PCL_INCLUDE_DIRS})