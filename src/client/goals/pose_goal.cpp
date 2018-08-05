//
// Created by julian on 04.08.18.
//

#include "../../include/jaco_manipulation/client/goals/pose_goal.h"

PoseGoal::PoseGoal(const string &name) {
  goal.goal_type = name;
  goal.joint_goal.header.frame_id = planning_frame;
}

PoseGoal::PoseGoal(const string &name, const geometry_msgs::PoseStamped &goal_pose) {
  goal.goal_type = name;
  goal.joint_goal.header.frame_id = planning_frame;
  goal.pose_goal = goal_pose;
}

jaco_manipulation::PlanAndMoveArmGoal PoseGoal::getGoal() const {
  return goal;
}

void PoseGoal::setGoal(const geometry_msgs::PoseStamped &goal_pose) {
  goal.pose_goal = goal_pose;
}