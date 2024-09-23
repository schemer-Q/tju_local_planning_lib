aux_source_directory(src SRC_LIST)
message(STATUS "SRC_LIST: ${SRC_LIST}")
add_library(trunk_perception_log SHARED ${SRC_LIST})
target_link_libraries(trunk_perception_log
  PkgConfig::JIAN)

########################
# install
########################
install(TARGETS trunk_perception_log
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
