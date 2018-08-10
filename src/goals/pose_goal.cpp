//
// Created by julian on 04.08.18.
//

#include <jaco_manipulation/goals/pose_goal.h>
#include <ros/console.h>

using namespace jaco_manipulation::goals;

PoseGoal::PoseGoal(const geometry_msgs::PoseStamped &goal_pose, const std::string &description) {
  description_ = description;

  goal_.goal_type = "pose";
  goal_.pose_goal = goal_pose;
  goal_.pose_goal.header.frame_id = planning_frame_;

  adjustHeight();

  ROS_INFO_STREAM("Attempt : Move to " << info());
}

void PoseGoal::adjustHeight() {
  auto &height = goal_.pose_goal.pose.position.z;
  if (height < min_height_) {
    ROS_WARN("Goal Fix: Height to low. Corrected to minimum height");
    height = min_height_;
  }
}

jaco_manipulation::PlanAndMoveArmGoal PoseGoal::goal() const {
  return Goal::goal();
}
