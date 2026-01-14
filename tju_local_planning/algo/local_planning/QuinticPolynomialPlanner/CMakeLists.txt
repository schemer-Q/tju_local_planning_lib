
#自动查找指定目录中的所有源文件，并将这些文件的路径添加到指定的变量中
aux_source_directory(QuinticPolynomial SRCS)
aux_source_directory(HybridAStar SRCS)
aux_source_directory(hybrid_astar SRCS)

add_library(tju_algo_local_planner SHARED 
  ${SRCS}
  local_trajectory_planner_manager.cpp)

target_link_libraries(tju_algo_local_planner
  tju_local_planning_log
  tju_local_planning_common
  yaml-cpp
)

########################
# install
########################
# 这里只需要该项目名称即可 其他不用改
install(TARGETS tju_algo_local_planner
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
