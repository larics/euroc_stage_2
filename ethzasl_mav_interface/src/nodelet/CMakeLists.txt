add_library(${PROJECT_NAME}_nodelet nodelet.cpp)
add_dependencies(${PROJECT_NAME}_nodelet ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_nodelet ${catkin_LIBRARIES} ${PROJECT_NAME})
SET_TARGET_PROPERTIES(${PROJECT_NAME}_nodelet PROPERTIES COMPILE_FLAGS "-std=c++11")