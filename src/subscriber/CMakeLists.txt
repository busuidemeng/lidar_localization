file(GLOB ALL_SOURCE "*.cpp")

add_library(SUBSCRIBER SHARED
  ${ALL_SOURCE}
)

target_link_libraries(SUBSCRIBER
  ${catkin_LIBRARIES}
)

add_dependencies(SUBSCRIBER 
  ${catkin_EXPORTED_TARGETS} 
  ${PROJECT_NAME}_generate_messages_cpp
)