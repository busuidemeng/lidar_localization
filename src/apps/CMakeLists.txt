find_package(glog REQUIRED)

include_directories(${glog_INCLUDE_DIRS})

# data_pretreat_node
add_executable(data_pretreat_node
  data_pretreat_node.cpp
)

add_dependencies(data_pretreat_node 
  ${catkin_EXPORTED_TARGETS} 
  ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(data_pretreat_node
  ${catkin_LIBRARIES}
  DATA_PRETREAT
  glog
)

if(BUILD_MAPPING)
  add_subdirectory(mapping)
else()
  if(USE_KALMAN)
    add_subdirectory(filtering)
  else()
    add_subdirectory(matching)
  endif()
endif()

# hk_data_pretreat_node
add_executable(hk_data_pretreat_node
  hk_data_pretreat_node.cpp
)

add_dependencies(hk_data_pretreat_node 
  ${catkin_EXPORTED_TARGETS} 
  ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(hk_data_pretreat_node
  ${catkin_LIBRARIES}
  DATA_PRETREAT
  glog
)
