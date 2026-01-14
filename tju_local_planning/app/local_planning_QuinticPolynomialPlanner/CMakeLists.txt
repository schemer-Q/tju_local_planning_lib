# 收集源文件   自动查找 src 目录中的所有源文件
aux_source_directory(src SRC_FILES)

# 声明一个库 创建一个名为 suntae_perception_app_local_planner 的共享库
add_library(tju_app_local_planner SHARED 
  ${SRC_FILES}
)

# 链接其他需要的库 指定了目标库需要链接的其他库
target_link_libraries(tju_app_local_planner
  # 将原来的库名称替换为正确的库名称
  tju_local_planning_common
  tju_local_planning_log
  tju_algo_local_planner
  yaml-cpp
  stdc++fs
)

########################
# install
########################
install(TARGETS tju_app_local_planner
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
