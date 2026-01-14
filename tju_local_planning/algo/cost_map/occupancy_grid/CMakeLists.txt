cmake_minimum_required(VERSION 3.12)

# Collect all .cpp/.c files in this directory (auto-refresh when files change)
file(GLOB SRCS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/*.c"
)

add_library(tju_costmap_occupancy STATIC ${SRCS})
target_include_directories(tju_costmap_occupancy PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Link to common libraries if available (adjust as needed)
target_link_libraries(tju_costmap_occupancy PUBLIC
  tju_local_planning_common
)

install(TARGETS tju_costmap_occupancy
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)