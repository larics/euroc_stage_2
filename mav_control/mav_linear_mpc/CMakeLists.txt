cmake_minimum_required(VERSION 2.8.3)
project(mav_linear_mpc)

#Choose between CVXGEN and FORCES!
#set(SOLVER_NAME "CVXGEN")
set(SOLVER_NAME "FORCES")

if("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "x86_64" AND NOT APPLE)
  add_definitions(-fPIC)
endif()

if(SOLVER_NAME STREQUAL "CVXGEN")
 add_library(${PROJECT_NAME}_cvxgen_solver
  src/CVXGEN_Solver/ldl.c
  src/CVXGEN_Solver/solver.c
  src/CVXGEN_Solver/matrix_support.c
  src/CVXGEN_Solver/util.c
  src/CVXGEN_Solver/variables_definition.c
 )

add_definitions(-DUseCVXGENSolver)
	include_directories(src/CVXGEN_Solver)
endif()

if(SOLVER_NAME STREQUAL "FORCES")
 add_library(${PROJECT_NAME}_forces_solver
  src/FireFlyOffsetFreeMPC.c
 )
 add_definitions(-DUseForcesSolver)
endif()

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mav_msgs
  roscpp
  sensor_msgs
  cmake_modules
  message_generation
  dynamic_reconfigure
  tf
  mav_control_interface
  tf_conversions
  )

find_package(Eigen REQUIRED)

add_message_files(
  FILES
  ObserverState.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  )

generate_dynamic_reconfigure_options(
  cfg/LinearMPC.cfg
  cfg/PDAttitude.cfg
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  )

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES
  CATKIN_DEPENDS geometry_msgs mav_msgs roscpp sensor_msgs mav_control_interface message_runtime
  DEPENDS Eigen
  )


add_library(mav_linear_mpc
  src/linear_mpc.cpp
  src/steady_state_calculation.cpp
  src/KF_disturbance_observer.cpp
  )
add_dependencies(mav_linear_mpc ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS}})
SET_TARGET_PROPERTIES(mav_linear_mpc PROPERTIES COMPILE_FLAGS "-std=c++11")

add_library(PD_attitude_controller src/PD_attitude_controller.cpp)
target_link_libraries(PD_attitude_controller ${catkin_LIBRARIES})
add_dependencies(PD_attitude_controller ${catkin_EXPORTED_TARGETS})
SET_TARGET_PROPERTIES(PD_attitude_controller PROPERTIES COMPILE_FLAGS "-std=c++11")

if(SOLVER_NAME STREQUAL "FORCES")
  target_link_libraries(mav_linear_mpc ${PROJECT_NAME}_forces_solver ${catkin_LIBRARIES})
endif()

if(SOLVER_NAME STREQUAL "CVXGEN")
  target_link_libraries(mav_linear_mpc ${PROJECT_NAME}_cvxgen_solver ${catkin_LIBRARIES})
endif()

add_executable(mav_linear_mpc_node_controller_only src/linear_mpc_node_controller_only.cpp)
target_link_libraries(mav_linear_mpc_node_controller_only mav_linear_mpc ${catkin_LIBRARIES})
add_dependencies(mav_linear_mpc_node_controller_only ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
SET_TARGET_PROPERTIES(mav_linear_mpc_node_controller_only PROPERTIES COMPILE_FLAGS "-std=c++11")

add_executable(mav_linear_mpc_node src/linear_mpc_node.cpp)
target_link_libraries(mav_linear_mpc_node mav_linear_mpc ${catkin_LIBRARIES})
add_dependencies(mav_linear_mpc_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
SET_TARGET_PROPERTIES(mav_linear_mpc_node PROPERTIES COMPILE_FLAGS "-std=c++11")

add_executable(PD_attitude_controller_node src/PD_attitude_controller_node.cpp)
target_link_libraries(PD_attitude_controller_node PD_attitude_controller ${catkin_LIBRARIES})
add_dependencies(PD_attitude_controller_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
SET_TARGET_PROPERTIES(PD_attitude_controller_node PROPERTIES COMPILE_FLAGS "-std=c++11")
