cmake_minimum_required(VERSION 2.8.3)
project(vehicle_monitor_library)

add_definitions(-std=c++11)


## Find catkin macros and libraries
find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  tf
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system filesystem)

find_package(octomap REQUIRED)
find_package(dynamicEDT3D REQUIRED)

find_package(Eigen REQUIRED)


catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS tf
  DEPENDS Eigen
)

###########
## Build ##
###########


include_directories(include)

include_directories(
    ${Eigen_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${OCTOMAP_INCLUDE_DIRS}
    ${DYNAMICEDT3D_INCLUDE_DIRS}
)


## Declare a cpp library

set(VML_SRC
    
    # Base Classes
    
	src/BaseConstraintChecker.cpp
	src/BaseVelocityEstimator.cpp
	src/BoundingVolume.cpp
	src/MotionCaptureSystemFrame.cpp
	#src/MotionCaptureSystemFrameElement.cpp
	src/Vehicle.cpp
	src/VehicleMonitor.cpp

    # Checkers
    
    src/AttitudeConstraintChecker.cpp
    src/CollisionConstraintChecker.cpp
    src/EmergencyButtonConstraintChecker.cpp
    src/MotionConstraintChecker.cpp
    src/OutOfSpaceConstraintChecker.cpp
    src/VelocityConstraintChecker.cpp
	
	# Checkers plugins
	
	src/SimpleVelocityEstimator.cpp
	

)

add_library(vehicle_monitor_library ${VML_SRC})

## Declare a cpp executable
# add_executable(vehicle_monitor_library_node src/vehicle_monitor_library_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(vehicle_monitor_library octomap dynamicEDT3D)

## Specify libraries to link a library or executable target against

link_directories(
#    ${OCTOMAP_LIBRARY_DIRS}
    ${DYNAMICEDT3D_LIBRARY_DIRS} 
)

target_link_libraries(vehicle_monitor_library
    ${catkin_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
	${Boost_FILESYSTEM_LIBRARY}
    ${OCTOMAP_LIBRARIES}
    ${DYNAMICEDT3D_LIBRARIES}
)

#############
## Install ##
#############


## Mark executables and/or libraries for installation
install(TARGETS vehicle_monitor_library
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/vehicle_monitor_library/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.hpp"
   PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_vehicle_monitor_library.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
