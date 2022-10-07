find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})

file(GLOB ALL_SOURCE "*.cpp")

add_library(TF_LISTENER SHARED
  ${ALL_SOURCE}
)

target_link_libraries(TF_LISTENER
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)