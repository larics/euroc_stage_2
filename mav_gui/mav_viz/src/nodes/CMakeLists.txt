add_executable(hexacopter_publisher hexacopter_publisher.cpp)
target_link_libraries(hexacopter_publisher mav_viz ${catkin_LIBRARIES})

add_executable(leica_publisher leica_publisher.cpp)
target_link_libraries(leica_publisher mav_viz ${catkin_LIBRARIES})
