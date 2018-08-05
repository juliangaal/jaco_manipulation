//
// Created by julian on 04.08.18.
//

#include <jaco_manipulation/client/goals/pose_goal.h>

using namespace jaco_manipulation::client::goals;

PoseGoal::PoseGoal(const geometry_msgs::PoseStamped &goal_pose) {
  goal.goal_type = "pose";
  goal.pose_goal = goal_pose;
  goal.pose_goal.header.frame_id = planning_frame;
}

jaco_manipulation::PlanAndMoveArmGoal PoseGoal::getGoal() const {
  return goal;
}