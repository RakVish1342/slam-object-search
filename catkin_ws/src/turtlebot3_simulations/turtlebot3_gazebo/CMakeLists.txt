################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.0)
project(turtlebot3_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## User defined valriables
set(EIGEN_DIRS "/usr/include/eigen3")
set(ROS_BASE_DIR_KINETIC "/opt/ros/kinetic/include")
set(ROS_BASE_DIR_MELODIC "/opt/ros/melodic/include")

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  gazebo_ros
  aruco_ros
  aruco_msgs
)

find_package(gazebo REQUIRED)

find_package(GTSAMCMakeTools)
find_package(GTSAM REQUIRED) # Uses installed package

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

#find_package(Boost 1.45.0 COMPONENTS *boost libraries here*) 
#find_package(Boost REQUIRED COMPONENTS *boost libraries here*)
find_package(Boost REQUIRED COMPONENTS serialization timer)

if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
    #add_executable(progname file1.cxx file2.cxx)
    #target_link_libraries(progname ${Boost_LIBRARIES})
endif()

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs tf gazebo_ros aruco_ros aruco_msgs
  DEPENDS gazebo
)

################################################################################
# Build
################################################################################
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${EIGEN_DIRS}
  ${ROS_BASE_DIR_KINETIC}
  ${ROS_BASE_DIR_MELODIC}
  ${GTSAM_INCLUDE_DIR}
)


add_library(odomLib src/OdometryExample.cpp)


add_executable(turtlebot3_drive src/turtlebot3_drive.cpp)
add_executable(motion_model src/motionModel.cpp)
add_executable(ekf_lidarTest src/ekfLidarTest.cpp)
add_executable(ekf_Test src/ekfTest.cpp)
add_executable(ekf_TestMoving src/ekfTestMoving.cpp)
add_executable(ekf_TestMovingAruco src/ekfTestMovingAruco.cpp)
add_executable(ekf src/ekf.cpp)
add_executable(control_loop src/control.cpp)
add_executable(ekf_sensorMle src/ekfSensorMle.cpp)
add_executable(gtsamExe src/OdometryExample.cpp)

add_dependencies(turtlebot3_drive ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(turtlebot3_drive ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
#target_link_libraries(motion_model ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(motion_model ${catkin_LIBRARIES})
target_link_libraries(ekf_lidarTest ${catkin_LIBRARIES})
target_link_libraries(ekf_Test ${catkin_LIBRARIES})
target_link_libraries(ekf_TestMoving ${catkin_LIBRARIES})
target_link_libraries(ekf_TestMovingAruco ${catkin_LIBRARIES})
target_link_libraries(ekf ${catkin_LIBRARIES})
target_link_libraries(control_loop ${catkin_LIBRARIES})
target_link_libraries(ekf_sensorMle ${catkin_LIBRARIES})

target_link_libraries(odomLib gtsam)
target_link_libraries(gtsamExe odomLib)

target_link_libraries(gtsamExe ${Boost_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS turtlebot3_drive
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch models rviz worlds 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#message("***************************************")
#message("MY PATHS")
#message("***************************************")
#message(${EIGEN_DIRS})
#message("***************************************")
#message(${catkin_INCLUDE_DIRS})
#message("***************************************")
################################################################################
# Test
################################################################################
