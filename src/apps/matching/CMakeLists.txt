# lio_matching_node
add_executable(lio_matching_node
  lio_matching_node.cpp
)

add_dependencies(lio_matching_node 
  ${catkin_EXPORTED_TARGETS} 
  ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(lio_matching_node
  ${catkin_LIBRARIES}
  MATCHING_FRONT_END
  glog
)

# sliding_window_node
add_executable(sliding_window_node
  sliding_window_node.cpp
)

add_dependencies(sliding_window_node 
  ${catkin_EXPORTED_TARGETS} 
  ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(sliding_window_node
  ${catkin_LIBRARIES}
  MATCHING_BACK_END
  glog
)