# TESTS
# Test client for jaco_manipulation action server
add_executable(client_test src/client_test.cpp)
add_dependencies(client_test ${PROJECT_NAME}_generate_messages_cpp jaco2_manipulation)
target_link_libraries(client_test
                      ${catkin_LIBRARIES}
                      jaco2_manipulation)
# Test client for jaco_manipulation grasping
add_executable(client_grasp_test src/client_grasp_test.cpp)
add_dependencies(client_grasp_test ${PROJECT_NAME}_generate_messages_cpp jaco2_manipulation)
target_link_libraries(client_grasp_test
                      ${catkin_LIBRARIES}
                      jaco2_manipulation)