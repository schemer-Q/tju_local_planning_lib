aux_source_directory(src SRC_FILES)

add_library(trunk_perception_app_camera_preprocess SHARED ${SRC_FILES})

target_link_libraries(trunk_perception_app_camera_preprocess
  trunk_perception_common
  trunk_perception_log
  yaml-cpp
  ${OpenCV_LIBRARIES}
  stdc++fs
)

########################
# install
########################
install(TARGETS trunk_perception_app_camera_preprocess
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)