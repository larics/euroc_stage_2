set(MAV_VIZ_SRC 
  marker_group.cpp
  hexacopter_marker.cpp
  leica_marker.cpp
)

add_library(mav_viz ${MAV_VIZ_SRC})
target_link_libraries(mav_viz ${catkin_LIBRARIES})
