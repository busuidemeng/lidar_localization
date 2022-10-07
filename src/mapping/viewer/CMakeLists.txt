find_package(glog REQUIRED)

include_directories(${glog_INCLUDE_DIRS})

file(GLOB ALL_SOURCE "*.cpp")

add_library(MAPPING_VIEWER SHARED
  ${ALL_SOURCE}
)

target_link_libraries(MAPPING_VIEWER
  ${catkin_LIBRARIES}
  SUBSCRIBER
  PUBLISHER
  CLOUD_FILTER
  SENSOR_DATA
  TOOLS
  glog
)