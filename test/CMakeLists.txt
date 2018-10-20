# TESTS

add_library(csv_reader src/csv_reader.cpp ${Boost_INCLUDE_DIRS})
add_dependencies(csv_reader ${PROJECT_NAME}_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(csv_reader
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      grasp_pose_generator)

# Test client for jaco_manipulation action server
add_executable(client_test client_test.cpp)
add_dependencies(client_test ${PROJECT_NAME}_generate_messages_cpp jaco2_manipulation)
target_link_libraries(client_test
                      ${catkin_LIBRARIES}
                      jaco2_manipulation)
# Test client for jaco_manipulation grasping
add_executable(client_grasp_test client_grasp_test.cpp)
add_dependencies(client_grasp_test ${PROJECT_NAME}_generate_messages_cpp jaco2_manipulation)
target_link_libraries(client_grasp_test
                      ${catkin_LIBRARIES}
                      jaco2_manipulation)
# Baseline test
add_executable(baseline_test baseline_test.cpp src/baseline_csv_reader.cpp)
add_dependencies(baseline_test ${PROJECT_NAME}_generate_messages_cpp jaco2_manipulation csv_reader)
target_link_libraries(baseline_test
                      ${catkin_LIBRARIES}
                      jaco2_manipulation
                      csv_reader)
# anchoring test
add_executable(anchoring_test anchoring_test.cpp src/baseline_csv_reader.cpp)
add_dependencies(anchoring_test ${PROJECT_NAME}_generate_messages_cpp jaco2_manipulation anchor_msgs)
target_link_libraries(anchoring_test
                      ${catkin_LIBRARIES}
                      jaco2_manipulation
                      csv_reader)
# obstacle anchoring test
#add_executable(obstacle_anchoring_test src/obstacle_anchoring_test.cpp src/anchoring_base_test.cpp src/baseline_csv_reader.cpp)
#add_dependencies(obstacle_anchoring_test ${PROJECT_NAME}_generate_messages_cpp jaco2_manipulation anchor_msgs)
#target_link_libraries(obstacle_anchoring_test
#                      ${catkin_LIBRARIES}
#                      jaco2_manipulation
#                      csv_reader)