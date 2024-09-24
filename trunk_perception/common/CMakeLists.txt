aux_source_directory(data_manager/src SRC_LIST)
message(STATUS "SRC_LIST: ${SRC_LIST}")
add_library(trunk_perception_common SHARED ${SRC_LIST})

target_link_libraries(trunk_perception_common PUBLIC
  trunk_perception_log
  yaml-cpp
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES})

########################
# install
########################
install(TARGETS trunk_perception_common
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)