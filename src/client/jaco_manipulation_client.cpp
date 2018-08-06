//
// Created by julian on 05.08.18.
//

#include <jaco_manipulation/client/jaco_manipulation_client.h>
#include <jaco_manipulation/client/goals/grasp_goal.h>

using namespace jaco_manipulation::client;

JacoManipulationClient::JacoManipulationClient() : client_("plan_and_move_arm", true) {
  client_.waitForServer();
}

void JacoManipulationClient::moveTo(const std::string &moveit_goal, const std::string &description) {
  goals::MoveItGoal goal(moveit_goal, description);
  execute(goal);
}

void JacoManipulationClient::moveTo(const geometry_msgs::PoseStamped &pose_goal, 
                                    const std::string &description) {
  goals::PoseGoal goal(pose_goal, description);
  execute(goal);
}

void JacoManipulationClient::moveTo(const sensor_msgs::JointState &joint_goal, const std::string &description) {
  goals::JointGoal goal(joint_goal.position, description);
  execute(goal);
}

void JacoManipulationClient::grasp(const goals::grasp_helper::GraspPose &grasp_pose_goal, const std::string &description) {
  goals::GraspGoal goal(grasp_pose_goal, description);
  execute(goal);
}

void JacoManipulationClient::execute(const goals::Goal &goal_wrapper) {
  const auto& goal = goal_wrapper.getGoal();

  client_.sendGoal(goal);
  client_.waitForResult();

  if (client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_SUCCESS("Status : Move to " + goal_wrapper.getDescription()+ " succeeded.");
  } else {
    ROS_ERROR_STREAM("Status : Move to " <<  goal_wrapper.getDescription()<< " failed.");
  }
}
