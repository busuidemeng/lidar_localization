# front_end_node
add_executable(front_end_node
  front_end_node.cpp
)

add_dependencies(front_end_node 
  ${catkin_EXPORTED_TARGETS} 
  ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(front_end_node
  ${catkin_LIBRARIES}
  MAPPING_FRONT_END
  glog
)

# lio_back_end_node
add_executable(lio_back_end_node
lio_back_end_node.cpp
)

add_dependencies(lio_back_end_node 
  ${catkin_EXPORTED_TARGETS} 
  ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(lio_back_end_node
  ${catkin_LIBRARIES}
  MAPPING_BACK_END
  glog
)

# loop_closing_node
add_executable(loop_closing_node
  loop_closing_node.cpp
)

add_dependencies(loop_closing_node 
  ${catkin_EXPORTED_TARGETS} 
  ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(loop_closing_node
  ${catkin_LIBRARIES}
  MAPPING_LOOP_CLOSING
  glog
)

# viewer_node
add_executable(viewer_node
viewer_node.cpp
)

add_dependencies(viewer_node 
  ${catkin_EXPORTED_TARGETS} 
  ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(viewer_node
  ${catkin_LIBRARIES}
  MAPPING_VIEWER
  glog
)

# lio_back_end_sim_node
add_executable(lio_back_end_sim_node
lio_back_end_sim_node.cpp
)

add_dependencies(lio_back_end_sim_node 
  ${catkin_EXPORTED_TARGETS} 
  ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(lio_back_end_sim_node
  ${catkin_LIBRARIES}
  MAPPING_BACK_END
  glog
)

# simulator_node
add_executable(simulator_node
  simulator_node.cpp
)

add_dependencies(simulator_node 
  ${catkin_EXPORTED_TARGETS} 
  ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(simulator_node
  ${catkin_LIBRARIES}
  SIMULATOR
  glog
)

# hk_lio_back_end_node
add_executable(hk_lio_back_end_node
hk_lio_back_end_node.cpp
)

add_dependencies(hk_lio_back_end_node 
  ${catkin_EXPORTED_TARGETS} 
  ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(hk_lio_back_end_node
  ${catkin_LIBRARIES}
  MAPPING_BACK_END
  glog
)
