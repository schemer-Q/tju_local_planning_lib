aux_source_directory(common TRACKER_SRCS)
aux_source_directory(kalman_filter TRACKER_SRCS)
aux_source_directory(matcher TRACKER_SRCS)
aux_source_directory(matcher/hungarian TRACKER_SRCS)
aux_source_directory(matcher/distance_measurement TRACKER_SRCS)
aux_source_directory(matcher/graph TRACKER_SRCS)
aux_source_directory(tracker_method TRACKER_SRCS)
aux_source_directory(tracker_pipeline TRACKER_SRCS)

add_library(trunk_perception_algo_track SHARED ${TRACKER_SRCS})
target_link_libraries(trunk_perception_algo_track
  trunk_perception_log
  trunk_perception_common
  yaml-cpp
  ${PCL_LIBRARIES}
)

########################
# install
########################
install(TARGETS trunk_perception_algo_track
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)