aux_source_directory(src SRC_FILES)
aux_source_directory(algo/src ALGO_FILES)
aux_source_directory(algo/tracker/src ALGO_FILES)

add_library(trunk_perception_app_lane_detection_post SHARED ${SRC_FILES} ${ALGO_FILES})

target_link_libraries(trunk_perception_app_lane_detection_post
  trunk_perception_common
  trunk_perception_log
  yaml-cpp
  ${OpenCV_LIBRARIES}
  stdc++fs
)

########################
# install
########################
install(TARGETS trunk_perception_app_lane_detection_post
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)