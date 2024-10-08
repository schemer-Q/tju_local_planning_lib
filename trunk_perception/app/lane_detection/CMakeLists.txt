aux_source_directory(src SRC_LIST)
aux_source_directory(algo/src ALGO_SRC_LIST)
add_library(trunk_perception_app_lane_detection SHARED ${SRC_LIST} ${ALGO_SRC_LIST})
target_link_libraries(trunk_perception_app_lane_detection
  trunk_perception_log
  trunk_perception_common
  yaml-cpp
  ${OpenCV_LIBRARIES}
  ${SDKBevLaneConeDetHighway_LIBRARIES}
)

########################
# install
########################
install(TARGETS trunk_perception_app_lane_detection
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)