add_library(${PROJECT_NAME}_c asctecCommIntf.c)
SET_TARGET_PROPERTIES(${PROJECT_NAME}_c PROPERTIES COMPILE_FLAGS "-std=c99")

add_library(${PROJECT_NAME}_uart uart.cpp)
target_link_libraries(${PROJECT_NAME}_uart ${catkin_LIBRARIES})
SET_TARGET_PROPERTIES(${PROJECT_NAME}_uart PROPERTIES COMPILE_FLAGS "-std=c++11")

add_library(${PROJECT_NAME} aci.cpp timesync_base.cpp simple_time_synchronizer.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PROJECT_NAME}_c)
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-std=c++11")
