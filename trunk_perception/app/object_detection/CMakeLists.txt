aux_source_directory(src SRC_LIST)
add_library(trunk_perception_app_object_detection SHARED ${SRC_LIST})
target_link_libraries(trunk_perception_app_object_detection
  trunk_perception_log
  trunk_perception_common
  yaml-cpp
  ${DetectionNetSDK_LIBRARIES}
  ${PCL_LIBRARIES}
  stdc++fs
)

########################
# install
########################
install(TARGETS trunk_perception_app_object_detection
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)