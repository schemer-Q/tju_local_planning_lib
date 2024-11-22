aux_source_directory(jcp_ground_detection SRCS)

add_library(trunk_perception_algo_ground_detection SHARED 
  ${SRCS}
  ground_detection_manager.cpp)

target_link_libraries(trunk_perception_algo_ground_detection
  trunk_perception_log
  trunk_perception_common
  yaml-cpp
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

########################
# install
########################
install(TARGETS trunk_perception_algo_ground_detection
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)