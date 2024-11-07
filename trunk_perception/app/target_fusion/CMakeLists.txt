aux_source_directory(src SRC_FILES)
aux_source_directory(target_fusion_impl/src SRC_FILES)
aux_source_directory(data_fusion/src SRC_FILES)
aux_source_directory(data_associate/src SRC_FILES)
aux_source_directory(utils/src SRC_FILES)

add_library(trunk_perception_app_target_fusion SHARED ${SRC_FILES})

target_link_libraries(trunk_perception_app_target_fusion
  trunk_perception_common
  trunk_perception_log
  yaml-cpp
  stdc++fs
  ${PCL_LIBRARIES}
)

########################
# install
########################
install(TARGETS trunk_perception_app_target_fusion
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)