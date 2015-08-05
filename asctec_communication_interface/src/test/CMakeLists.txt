catkin_add_gtest(test_time_synchronization test_time_synchronization.cpp)
target_link_libraries(test_time_synchronization ${PROJECT_NAME})
SET_TARGET_PROPERTIES(test_time_synchronization PROPERTIES COMPILE_FLAGS "-std=c++11")

catkin_add_gtest(test_misc test_misc.cpp)
target_link_libraries(test_misc ${PROJECT_NAME})
SET_TARGET_PROPERTIES(test_misc PROPERTIES COMPILE_FLAGS "-std=c++11")