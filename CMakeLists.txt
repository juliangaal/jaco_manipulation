cmake_minimum_required(VERSION 2.8.3)
project(jaco_manipulation)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(COMPILER "UNKNOWN")

if(CMAKE_COMPILER_IS_GNUCXX)
  set(COMPILER "G")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "AppleClang")
  set(COMPILER "Apple Clang")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "AppleClang")
  set(COMPILER "Clang")
endif()

if("${COMPILER}" STREQUAL "UNKNOWN")
  message(FATAL_ERROR "Invalid Compiler")
endif()

message(STATUS "${COMPILER}++ Version : " ${CMAKE_CXX_COMPILER_VERSION})
message(STATUS "${COMPILER}++ Location: " ${CMAKE_CXX_COMPILER})

### Require out-of-source builds
file(TO_CMAKE_PATH "${PROJECT_BINARY_DIR}/CMakeLists.txt" LOC_PATH)
if(EXISTS "${LOC_PATH}")
  message(FATAL_ERROR "You cannot build in a source directory (or any directory with a CMakeLists.txt file).
          Please make a build subdirectory. Feel free to remove CMakeCache.txt and CMakeFiles.")
endif()

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
             rospy
             std_msgs
             tf
             message_generation
             wpi_jaco_msgs)

find_package(Boost REQUIRED)

add_message_files(
    FILES
    BoundingBox.msg
    MoveItGoal.msg
    MoveItConfig.msg
    PlanningScene.msg
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
    CATKIN_DEPENDS  roscpp rospy actionlib actionlib_msgs cmake_modules geometry_msgs moveit_core
                    moveit_ros_planning_interface pluginlib roscpp std_msgs tf message_runtime wpi_jaco_msgs)

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
add_library(jaco2_manipulation ${SERVER_SOURCES} ${CLIENT_SOURCES})
add_dependencies(jaco2_manipulation ${PROJECT_NAME}_generate_messages_cpp
                  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(jaco2_manipulation PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(jaco2_manipulation
                      ${catkin_LIBRARIES}
                      grasp_pose_generator)

add_library(grasp_pose_generator src/grasps/grasp_pose_generator.cpp)
target_include_directories(grasp_pose_generator PUBLIC include ${catkin_INCLUDE_DIRS})
add_dependencies(grasp_pose_generator ${PROJECT_NAME}_generate_messages_cpp)

add_executable(jaco_manipulation_server src/${PROJECT_NAME}_node.cpp)
target_include_directories(jaco_manipulation_server PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(jaco_manipulation_server
                      ${catkin_LIBRARIES}
                      jaco2_manipulation)

install(TARGETS jaco2_manipulation grasp_pose_generator
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
