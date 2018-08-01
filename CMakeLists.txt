cmake_minimum_required(VERSION 2.8.3)
project(jaco_manipulation)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  cmake_modules
  geometry_msgs
  moveit_core
  moveit_ros_planning_interface
  pluginlib
  roscpp
  std_msgs
  tf
  message_generation
  wpi_jaco_msgs
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GenerateGraspPoses.srv
  GeneratePutDownPoints.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  PlanAndMoveArm.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES jaco2_manipulation grasp_pose_generator
  CATKIN_DEPENDS actionlib actionlib_msgs cmake_modules geometry_msgs moveit_core moveit_ros_planning_interface pluginlib roscpp std_msgs tf message_runtime wpi_jaco_msgs
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library --> DoroManipulation
add_library(jaco2_manipulation
    src/jaco_manipulation.cpp
)

add_library(grasp_pose_generator
    src/grasp_pose_generator.cpp
)

#add_library(put_down_points_generator
#    src/put_down_points_generator.cpp
#)

# Test client for grasp pose generator
add_executable(gpg_client src/gpg_client.cc)

# Test client for jaco_manipulation
add_executable(pam_client src/jaco_manipulation_test.cc)

add_executable(manipulation_server src/manipulation_services_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(gpg_client jaco_manipulation_generate_messages_cpp)
add_dependencies(jaco2_manipulation jaco_manipulation_generate_messages_cpp)
add_dependencies(grasp_pose_generator jaco_manipulation_generate_messages_cpp)
add_dependencies(pam_client jaco_manipulation_generate_messages_cpp)
#add_dependencies(put_down_points_generator jaco_manipulation_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(gpg_client
  ${catkin_LIBRARIES}
)

target_link_libraries(jaco2_manipulation
  ${catkin_LIBRARIES}
)

target_link_libraries(grasp_pose_generator
  ${catkin_LIBRARIES}
)

#target_link_libraries(put_down_points_generator
#  ${catkin_LIBRARIES}
#)

target_link_libraries(manipulation_server
  ${catkin_LIBRARIES}
  grasp_pose_generator
  jaco2_manipulation
#  put_down_points_generator
)

target_link_libraries(pam_client
  ${catkin_LIBRARIES}
)
