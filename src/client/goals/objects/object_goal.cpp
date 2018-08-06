//
// Created by julian on 8/6/18.
//

#include <jaco_manipulation/client/goals/objects/object_goal.h>
#include <ros/console.h>

using namespace jaco_manipulation::client::goals::objects;

ObjectGoal::ObjectGoal(const grasp_helper::GraspPose &grasp_pose_goal, const std::string &description) {
  ROS_INFO("----");
  ROS_INFO_STREAM("Attempt: Move to " << description);
  description_ = description;

  goal_.goal_type = "goal";
  goal_.pose_goal.pose.position.x = grasp_pose_goal.x;
  goal_.pose_goal.pose.position.y = grasp_pose_goal.y;
  goal_.pose_goal.pose.position.z = grasp_pose_goal.z;

  PoseGoal::adjustHeight();

  goal_.pose_goal.pose.orientation.x = default_rot_x_;
  goal_.pose_goal.pose.orientation.y = default_rot_y_;
  goal_.pose_goal.pose.orientation.z = default_rot_z_;
  goal_.pose_goal.pose.orientation.w = default_orientation_;

  if (default_orientation_ != grasp_pose_goal.rotation && grasp_pose_goal.rotation != 0.0)
    goal_.pose_goal.pose.orientation.w = grasp_pose_goal.rotation;

  goal_.pose_goal.header.frame_id = planning_frame_;
}

ObjectGoal::ObjectGoal(const grasp_helper::Object &object_goal, const std::string &description) {
  ROS_INFO("----");
  ROS_INFO_STREAM("Attempt: Move to " << description << ": " << object_goal.description);

  description_ = description;

  goal_.goal_type = "goal";

  ObjectGoal::adjustPoseToCenterOfObject(object_goal);
  ObjectGoal::adjustHeight();

  goal_.pose_goal.pose.orientation.x = default_rot_x_;
  goal_.pose_goal.pose.orientation.y = default_rot_y_;
  goal_.pose_goal.pose.orientation.z = default_rot_z_;
  goal_.pose_goal.pose.orientation.w = default_orientation_;
  goal_.pose_goal.header.frame_id = planning_frame_;
}


void ObjectGoal::adjustPoseToCenterOfObject(const grasp_helper::Object &object) {
  ROS_INFO("Status  : Adjusting pose to center of object");
  goal_.pose_goal.pose.position.x = object.width * 1.5;
  goal_.pose_goal.pose.position.y = object.length * 1.5;
  goal_.pose_goal.pose.position.z = object.height;
}

void ObjectGoal::adjustHeight() {
  PoseGoal::adjustHeight();
}

jaco_manipulation::PlanAndMoveArmGoal ObjectGoal::getGoal() const {
  return PoseGoal::getGoal();
}

const std::string &ObjectGoal::getDescription() const {
  return PoseGoal::getDescription();
}
