//
// Created by julian on 04.08.18.
//

#include <jaco_manipulation/client/goals/pose_goal.h>
#include <ros/console.h>

using namespace jaco_manipulation::client::goals;

PoseGoal::PoseGoal(const geometry_msgs::PoseStamped &goal_pose, const std::string &description) {
  description_ = description;

  goal_.goal_type = "pose";
  goal_.pose_goal = goal_pose;
  goal_.pose_goal.header.frame_id = planning_frame_;

  ROS_INFO("----");
  ROS_INFO_STREAM("Attempt: Move to " << description_);
}

jaco_manipulation::PlanAndMoveArmGoal PoseGoal::getGoal() const {
  return Goal::getGoal();
}

const std::string &PoseGoal::getDescription() const {
  return Goal::getDescription();
}
