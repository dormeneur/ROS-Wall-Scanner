# Place this in: ~/catkin_ws/src/wall_scanner/CMakeLists.txt

cmake_minimum_required(VERSION 3.0.2)
project(wall_scanner)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  DetectionResult.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    rospy 
    std_msgs 
    sensor_msgs 
    geometry_msgs 
    visualization_msgs 
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install Python scripts
catkin_install_python(PROGRAMS
  scripts/electromagnetic_sensor_node.py
  scripts/capacitive_sensor_node.py
  scripts/detection_fusion_node.py
  scripts/visualization_node.py
  scripts/wall_simulator_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)