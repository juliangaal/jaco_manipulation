//
// Created by julian on 04.08.18.
//

#include <jaco_manipulation/client/goals/joint_goal.h>

using namespace jaco_manipulation::client::goals;

JointGoal::JointGoal(const std::vector<double> &joint_goal) {
  goal.goal_type = "joint_state";
  goal.joint_goal.position = joint_goal;
  goal.joint_goal.header.frame_id = planning_frame;
}

jaco_manipulation::PlanAndMoveArmGoal JointGoal::getGoal() const {
  return goal;
}
