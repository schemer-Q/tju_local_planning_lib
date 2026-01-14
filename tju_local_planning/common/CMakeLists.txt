aux_source_directory(data_manager/src SRC_LIST)
aux_source_directory(tools/src TOOLS_SRC_LIST)
set(SRC_LIST ${SRC_LIST} ${TOOLS_SRC_LIST})
# 添加所有.hpp文件
file(GLOB_RECURSE HEADERS_LIST "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
add_library(tju_local_planning_common SHARED ${SRC_LIST} ${HEADERS_LIST})

target_link_libraries(tju_local_planning_common PUBLIC
  tju_local_planning_log
  ${YAML_CPP_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  stdc++fs)

if (OpenMP_CXX_FOUND)
  target_link_libraries(tju_local_planning_common PUBLIC OpenMP::OpenMP_CXX)
  target_compile_options(tju_local_planning_common PRIVATE ${OpenMP_CXX_FLAGS})
endif()

########################
# install
########################
install(TARGETS tju_local_planning_common
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)