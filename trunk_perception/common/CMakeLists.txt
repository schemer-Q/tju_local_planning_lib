aux_source_directory(data_manager/src SRC_LIST)
# 添加所有.hpp文件
file(GLOB_RECURSE HEADERS_LIST "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
add_library(trunk_perception_common SHARED ${SRC_LIST} ${HEADERS_LIST})

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