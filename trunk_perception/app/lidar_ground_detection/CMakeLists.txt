aux_source_directory(src SRC_FILES)

add_library(trunk_perception_app_lidar_ground_detection SHARED ${SRC_FILES})

target_link_libraries(trunk_perception_app_lidar_ground_detection
  trunk_perception_algo_ground_detection
  trunk_perception_common
  trunk_perception_log
  yaml-cpp
  stdc++fs
  ${PCL_LIBRARIES}
)

########################
# install
########################
install(TARGETS trunk_perception_app_lidar_ground_detection
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)