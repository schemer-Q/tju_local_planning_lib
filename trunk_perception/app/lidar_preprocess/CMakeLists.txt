aux_source_directory(src SRC_FILES)

add_library(trunk_perception_app_lidar_preprocess SHARED ${SRC_FILES})

target_link_libraries(trunk_perception_app_lidar_preprocess
  trunk_perception_common
  trunk_perception_log
  yaml-cpp
  ${PCL_LIBRARIES}
)

########################
# install
########################
install(TARGETS trunk_perception_app_lidar_preprocess
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)