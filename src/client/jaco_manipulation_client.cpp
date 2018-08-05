//
// Created by julian on 05.08.18.
//

#include <jaco_manipulation/client/jaco_manipulation_client.h>

JacoManipulationClient::JacoManipulationClient() : client_("plan_and_move_arm", true) {
  client_.waitForServer();
}

void JacoManipulationClient::moveTo(const string &moveit_goal) {
  MoveItGoal goal(moveit_goal);
  execute(goal);
}

void JacoManipulationClient::moveTo(const PoseStamped &pose_goal) {
  PoseGoal goal(pose_goal);
  execute(goal);
}

void JacoManipulationClient::moveTo(const JointState &joint_goal) {
  JointGoal goal(joint_goal.position);
  execute(goal);
}

void JacoManipulationClient::execute(const Goal &goal_wrapper) {
  const auto& goal = goal_wrapper.getGoal();

  ROS_INFO("----");
  ROS_INFO_STREAM("Attempt: Move to " << goal.goal_type);

  client_.sendGoal(goal);
  client_.waitForResult();

  if (client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_SUCCESS("Status : Move to " + goal.goal_type + " succeeded.");
  } else {
    ROS_ERROR_STREAM("Status : Move to " << goal.goal_type << " failed.");
  }
}
