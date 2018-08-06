//
// Created by chitt on 8/6/18.
//

#include <jaco_manipulation/client/goals/grasp_goal.h>
#include <geometry_msgs/Quaternion.h>
#include <ros/console.h>

using namespace jaco_manipulation::client::goals;

GraspGoal::GraspGoal(const GraspPose &grasp_pose_goal) {
  constexpr double min_height = 0.175026;

  geometry_msgs::PoseStamped pose_goal;
  goal.goal_type = "pose";
  goal.pose_goal.pose.position.x = grasp_pose_goal.x;
  goal.pose_goal.pose.position.y = grasp_pose_goal.y;

  ROS_INFO("----");
  ROS_INFO_STREAM("Attempt: Move to " << goal.goal_type);

  if (grasp_pose_goal.z < min_height) {
    ROS_WARN_STREAM("Status : Grasp pose to low. Correcting height to " << min_height);
    goal.pose_goal.pose.position.z = min_height;
  } else {
    goal.pose_goal.pose.position.z = grasp_pose_goal.z;
  }

  geometry_msgs::Quaternion orientation_goal;
  orientation_goal.x = 0.033245;
  orientation_goal.y = -0.003230;
  orientation_goal.z = -0.737370;
  orientation_goal.w = grasp_pose_goal.rotation;
  goal.pose_goal.pose.orientation = orientation_goal;
  goal.pose_goal.header.frame_id = planning_frame;
}

jaco_manipulation::PlanAndMoveArmGoal GraspGoal::getGoal() const {
  return goal;
}