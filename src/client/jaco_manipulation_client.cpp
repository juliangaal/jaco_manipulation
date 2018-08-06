//
// Created by julian on 05.08.18.
//

#include <jaco_manipulation/client/jaco_manipulation_client.h>
#include <jaco_manipulation/client/goals/grasp_goal.h>

using namespace jaco_manipulation::client;

JacoManipulationClient::JacoManipulationClient() : client_("plan_and_move_arm", true) {
  client_.waitForServer();
}

void JacoManipulationClient::moveTo(const std::string &moveit_goal) {
  goals::MoveItGoal goal(moveit_goal);
  execute(goal);
}

void JacoManipulationClient::moveTo(const geometry_msgs::PoseStamped &pose_goal) {
  goals::PoseGoal goal(pose_goal);
  execute(goal);
}

void JacoManipulationClient::moveTo(const sensor_msgs::JointState &joint_goal) {
  goals::JointGoal goal(joint_goal.position);
  execute(goal);
}

void JacoManipulationClient::moveTo(const goals::GraspGoal::GraspPose &grasp_pose_goal) {
  goals::GraspGoal goal(grasp_pose_goal);
  execute(goal);
}

void JacoManipulationClient::execute(const goals::Goal &goal_wrapper) {
  const auto& goal = goal_wrapper.getGoal();

  client_.sendGoal(goal);
  client_.waitForResult();

  if (client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_SUCCESS("Status : Move to " + goal.goal_type + " succeeded.");
  } else {
    ROS_ERROR_STREAM("Status : Move to " << goal.goal_type << " failed.");
  }
}
