find_package(glog REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${glog_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

file(GLOB ALL_SOURCE "*.cpp")

add_library(FILTERING SHARED
  ${ALL_SOURCE}
)

target_link_libraries(FILTERING
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
  KALMAN_FILTER
  SENSOR_DATA
  SUBSCRIBER
  PUBLISHER
  TF_LISTENER
  CLOUD_FILTER
  REGISTRATION
  TOOLS
  glog
)