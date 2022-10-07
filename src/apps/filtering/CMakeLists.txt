# kitti_filtering_node
add_executable(kitti_filtering_node
  kitti_filtering_node.cpp
)

add_dependencies(kitti_filtering_node 
  ${catkin_EXPORTED_TARGETS} 
  ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(kitti_filtering_node
  ${catkin_LIBRARIES}
  FILTERING
  glog
)

# gnss_ins_sim_filtering_node
add_executable(gnss_ins_sim_filtering_node
  gnss_ins_sim_filtering_node.cpp
)

add_dependencies(gnss_ins_sim_filtering_node 
  ${catkin_EXPORTED_TARGETS} 
  ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(gnss_ins_sim_filtering_node
  ${catkin_LIBRARIES}
  FILTERING
  glog
)

# gnss_ins_sim_preprocess_node
add_executable(gnss_ins_sim_preprocess_node
  gnss_ins_sim_preprocess_node.cpp
)

add_dependencies(gnss_ins_sim_preprocess_node 
  ${catkin_EXPORTED_TARGETS} 
  ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(gnss_ins_sim_preprocess_node
  ${catkin_LIBRARIES}
  SIM_DATA_PREPROCESS
  glog
)
