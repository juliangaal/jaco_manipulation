//
// Created by julian on 06.08.18.
//

#include <jaco_manipulation/client/goals/objects/drop_goal.h>
#include <ros/console.h>

using namespace jaco_manipulation::client::goals;

DropGoal::DropGoal(const grasp_helper::GraspPose &drop_pose_goal, const std::string &description) {
  ROS_INFO("----");
  ROS_INFO_STREAM("Attempt: Move to " << description);
  description_ = description;

  goal_.goal_type = "drop_pose";
  goal_.pose_goal.pose.position.x = drop_pose_goal.x;
  goal_.pose_goal.pose.position.y = drop_pose_goal.y;
  goal_.pose_goal.pose.position.z = drop_pose_goal.z;

  GraspGoal::adjustHeight();

  goal_.pose_goal.pose.orientation.x = default_rot_x_;
  goal_.pose_goal.pose.orientation.y = default_rot_y_;
  goal_.pose_goal.pose.orientation.z = default_rot_z_;
  goal_.pose_goal.pose.orientation.w = default_orientation_;

  if (default_orientation_ != drop_pose_goal.rotation && drop_pose_goal.rotation != 0.0)
    goal_.pose_goal.pose.orientation.w = drop_pose_goal.rotation;

  goal_.pose_goal.header.frame_id = planning_frame_;
}

DropGoal::DropGoal(const grasp_helper::Object &object_goal, const std::string &description) {
  ROS_INFO("----");
  ROS_INFO_STREAM("Attempt: Move to " << description << ": " << object_goal.description);

  description_ = description;

  goal_.goal_type = "drop_pose";

  GraspGoal::adjustPoseToCenterOfObject(object_goal);
  GraspGoal::adjustHeight();

  goal_.pose_goal.pose.orientation.x = default_rot_x_;
  goal_.pose_goal.pose.orientation.y = default_rot_y_;
  goal_.pose_goal.pose.orientation.z = default_rot_z_;
  goal_.pose_goal.pose.orientation.w = default_orientation_;
  goal_.pose_goal.header.frame_id = planning_frame_;
}

jaco_manipulation::PlanAndMoveArmGoal DropGoal::getGoal() const {
  return Goal::getGoal();
}

const std::string &DropGoal::getDescription() const {
  return Goal::getDescription();
}
