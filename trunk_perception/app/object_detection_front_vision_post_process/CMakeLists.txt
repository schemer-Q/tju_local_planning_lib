aux_source_directory(src SRC_LIST)
add_library(trunk_perception_app_object_detection_front_vision_post_process SHARED ${SRC_LIST})
target_link_libraries(trunk_perception_app_object_detection_front_vision_post_process
  trunk_perception_log
  trunk_perception_common
  trunk_perception_algo_track
  yaml-cpp
  stdc++fs
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

########################
# install
########################
install(TARGETS trunk_perception_app_object_detection_front_vision_post_process
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)