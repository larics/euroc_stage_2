add_executable(example_node example_node.cpp)
target_link_libraries(example_node ${PROJECT_NAME} ${PROJECT_NAME}_uart)
SET_TARGET_PROPERTIES(example_node PROPERTIES COMPILE_FLAGS "-std=c++11")