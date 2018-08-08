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
        moveit_visual_tools
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
        sensor_msgs
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

set(SERVER_SOURCES
        src/server/jaco_boundaries.cpp
        src/server/jaco_manipulation_server.cpp
)

set(CLIENT_SOURCES
        src/client/jaco_manipulation_client.cpp
        src/client/goals/goal.cpp
        src/client/goals/objects/object_goal.cpp
        src/client/goals/objects/grasp_goal.cpp
        src/client/goals/objects/drop_goal.cpp
        src/client/goals/move_it_goal.cpp
        src/client/goals/pose_goal.cpp
        src/client/goals/joint_goal.cpp)

## Declare a cpp library --> DoroManipulation
add_library(jaco2_manipulation
        ${SERVER_SOURCES}
        ${CLIENT_SOURCES}
)

add_library(grasp_pose_generator
        src/grasp_pose_generator.cpp
)

# Test client for grasp pose generator action server
add_executable(grasp_pose_generator_client_test test/grasp_pose_generator_client_test.cpp)

# Test client for jaco_manipulation action server
add_executable(jaco_manipulation_client_test test/jaco_manipulation_client_test.cpp)

# Test client for jaco_manipulation grasping
add_executable(jaco_manipulation_client_grasp_test test/jaco_manipulation_client_grasp_test.cpp)

add_executable(manipulation_server src/manipulation_services_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(grasp_pose_generator_client_test jaco_manipulation_generate_messages_cpp)
add_dependencies(jaco2_manipulation jaco_manipulation_generate_messages_cpp)
add_dependencies(grasp_pose_generator jaco_manipulation_generate_messages_cpp)
add_dependencies(jaco_manipulation_client_test jaco_manipulation_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(grasp_pose_generator_client_test
        ${catkin_LIBRARIES}
)

target_link_libraries(jaco2_manipulation
        ${catkin_LIBRARIES}
)

target_link_libraries(grasp_pose_generator
        ${catkin_LIBRARIES}
)

target_link_libraries(manipulation_server
        ${catkin_LIBRARIES}
        grasp_pose_generator
        jaco2_manipulation
)

target_link_libraries(jaco_manipulation_client_test
        ${catkin_LIBRARIES}
        jaco2_manipulation
)

target_link_libraries(jaco_manipulation_client_grasp_test
        ${catkin_LIBRARIES}
        jaco2_manipulation
)
