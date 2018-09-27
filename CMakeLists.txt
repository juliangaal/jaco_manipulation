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
             wpi_jaco_msgs)

add_message_files(
    FILES
    BoundingBox.msg
    MoveItGoal.msg
    MoveItConfig.msg
    JacoDebug.msg)

## Generate actions in the 'action' folder
add_action_files(
    FILES
    PlanAndMoveArm.action)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    jaco_manipulation
    geometry_msgs
    sensor_msgs
    std_msgs
    actionlib_msgs)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES jaco2_manipulation grasp_pose_generator
    CATKIN_DEPENDS actionlib actionlib_msgs cmake_modules geometry_msgs moveit_core moveit_ros_planning_interface pluginlib roscpp std_msgs tf message_runtime wpi_jaco_msgs)

include_directories(
    include
    ${catkin_INCLUDE_DIRS})

set(SERVER_SOURCES
    src/visuals/moveit_visuals.cpp
    src/server/${PROJECT_NAME}_server.cpp)

set(CLIENT_SOURCES
    src/client/${PROJECT_NAME}_client.cpp
    src/goals/goal.cpp
    src/goals/vision_goal.cpp
    src/goals/grasp_goal.cpp
    src/goals/drop_goal.cpp
    src/goals/move_it_goal.cpp
    src/goals/pose_goal.cpp
    src/goals/joint_goal.cpp)

## Declare a cpp library --> DoroManipulation
add_library(jaco2_manipulation
            ${SERVER_SOURCES}
            ${CLIENT_SOURCES})

add_library(grasp_pose_generator
            src/grasps/grasp_pose_generator.cpp)

# Test client for jaco_manipulation action server
add_executable(client_test test/client_test.cpp)

# Test client for jaco_manipulation grasping
add_executable(client_grasp_test test/client_grasp_test.cpp)

add_executable(jaco_manipulation_server src/${PROJECT_NAME}_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(jaco2_manipulation ${PROJECT_NAME}_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(client_test ${PROJECT_NAME}_generate_messages_cpp jaco2_manipulation)
add_dependencies(grasp_pose_generator ${PROJECT_NAME}_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(jaco2_manipulation
                      ${catkin_LIBRARIES}
                      grasp_pose_generator)

target_link_libraries(jaco_manipulation_server
                      ${catkin_LIBRARIES}
                      jaco2_manipulation)

target_link_libraries(client_test
                      ${catkin_LIBRARIES}
                      jaco2_manipulation)

target_link_libraries(client_grasp_test
                      ${catkin_LIBRARIES}
                      jaco2_manipulation)
