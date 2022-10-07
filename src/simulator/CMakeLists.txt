file(GLOB ALL_SOURCE "*.cpp")

add_library(SIMULATOR SHARED
  ${ALL_SOURCE}
)

target_link_libraries(SIMULATOR
  ${catkin_LIBRARIES}
)

# add_dependencies(SIMULATOR 
#   ${catkin_EXPORTED_TARGETS} 
#   ${PROJECT_NAME}_generate_messages_cpp
# )