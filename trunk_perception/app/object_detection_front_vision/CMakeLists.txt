aux_source_directory(src SRC_LIST)
add_library(trunk_perception_app_object_detection_front_vision SHARED ${SRC_LIST})
target_link_libraries(trunk_perception_app_object_detection_front_vision
  trunk_perception_log
  trunk_perception_common
  yaml-cpp
  stdc++fs
  ${SDKVisDetR3DDconfFront_LIBRARIES}
)

########################
# install
########################
install(TARGETS trunk_perception_app_object_detection_front_vision
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)