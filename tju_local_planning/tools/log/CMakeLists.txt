aux_source_directory(src SRC_LIST)
message(STATUS "SRC_LIST: ${SRC_LIST}")
add_library(tju_local_planning_log SHARED ${SRC_LIST})
target_link_libraries(tju_local_planning_log
  PkgConfig::JIAN)



  # 构建测试可执行文件
add_executable(test_log src/test_log.cpp)
target_link_libraries(test_log
tju_local_planning_log
  PkgConfig::JIAN)



########################
# install
########################
install(TARGETS tju_local_planning_log
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)







