aux_source_directory(src SRC_LIST)
message(STATUS "SRC_LIST: ${SRC_LIST}")
add_library(t_log SHARED ${SRC_LIST})
target_link_libraries(t_log
  PkgConfig::JIAN)

########################
# install
########################
install(TARGETS t_log
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  DESTINATION include/tools
  FILES_MATCHING
  PATTERN "*.hpp"
  PATTERN "*.h"
  )
