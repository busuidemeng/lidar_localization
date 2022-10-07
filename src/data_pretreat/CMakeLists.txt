add_library(DATA_PRETREAT SHARED
  data_pretreat_flow.cpp
  hk_data_pretreat_flow.cpp
)

target_link_libraries(DATA_PRETREAT
  ${catkin_LIBRARIES}
  SENSOR_DATA
  SUBSCRIBER
  PUBLISHER
  TF_LISTENER
  SCAN_ADJUST
  TOOLS
)

if(USE_KALMAN)
  add_library(SIM_DATA_PREPROCESS SHARED
    gnss_ins_sim_preprocess_flow.cpp
  )

  target_link_libraries(SIM_DATA_PREPROCESS
    ${catkin_LIBRARIES}
    SENSOR_DATA
    SUBSCRIBER
    PUBLISHER
    TF_LISTENER
    SCAN_ADJUST
  )
endif()