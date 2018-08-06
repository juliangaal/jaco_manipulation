//
// Created by chitt on 8/6/18.
//

#include <jaco_manipulation/client/goals/grasp_goal.h>
#include <geometry_msgs/Quaternion.h>
#include <ros/console.h>

using namespace jaco_manipulation::client::goals;

GraspGoal::GraspGoal(const GraspPose &grasp_pose_goal, const std::string &description) {
  constexpr double min_height = 0.175026;

  description_ = description;

  geometry_msgs::PoseStamped pose_goal;
  goal_.goal_type = "pose";
  goal_.pose_goal.pose.position.x = grasp_pose_goal.x;
  goal_.pose_goal.pose.position.y = grasp_pose_goal.y;
  if (grasp_pose_goal.z < min_height) {
    ROS_WARN_STREAM("Status : Grasp pose to low. Correcting height to " << min_height);
    goal_.pose_goal.pose.position.z = min_height;
  } else {
    goal_.pose_goal.pose.position.z = grasp_pose_goal.z;
  }

  geometry_msgs::Quaternion orientation_goal;
  orientation_goal.x = 0.033245;
  orientation_goal.y = -0.003230;
  orientation_goal.z = -0.737370;
  orientation_goal.w = grasp_pose_goal.rotation;
  goal_.pose_goal.pose.orientation = orientation_goal;
  goal_.pose_goal.header.frame_id = planning_frame_;

  ROS_INFO("----");
  ROS_INFO_STREAM("Attempt: Move to " << description_);
}

jaco_manipulation::PlanAndMoveArmGoal GraspGoal::getGoal() const {
  return Goal::getGoal();
}

const std::string &GraspGoal::getDescription() const {
  return Goal::getDescription();
}
