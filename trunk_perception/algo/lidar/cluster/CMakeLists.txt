aux_source_directory(bev_ccl_cluster SRCS)

add_library(trunk_perception_algo_cluster SHARED 
  ${SRCS}
  cluster_manager.cpp)

target_link_libraries(trunk_perception_algo_cluster
  trunk_perception_log
  trunk_perception_common
  yaml-cpp
  ${PCL_LIBRARIES}
)

########################
# install
########################
install(TARGETS trunk_perception_algo_cluster
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)