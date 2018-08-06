//
// Created by julian on 04.08.18.
//

#include <jaco_manipulation/client/goals/joint_goal.h>
#include <ros/console.h>

using namespace jaco_manipulation::client::goals;

JointGoal::JointGoal(const std::vector<double> &joint_goal, const std::string &description) {
  description_ = description;
  goal_.goal_type = "joint_state";
  goal_.joint_goal.position = joint_goal;
  goal_.joint_goal.header.frame_id = planning_frame_;

  ROS_INFO("----");
  ROS_INFO_STREAM("Attempt: Move to " << description_);
}

jaco_manipulation::PlanAndMoveArmGoal JointGoal::getGoal() const {
  return Goal::getGoal();
}

const std::string &JointGoal::getDescription() const {
  return Goal::getDescription();
}
