//
// Created by julian on 8/6/18.
//

#include <jaco_manipulation/client/goals/objects/grasp_goal.h>
#include <ros/console.h>

using namespace jaco_manipulation::client::goals::objects;

GraspGoal::GraspGoal(const object_helper::LimitedPose &grasp_pose_goal, const std::string &description)
: ObjectGoal(grasp_pose_goal, description) {
  goal_.goal_type = "grasp_pose";

  ROS_INFO("----");
  ROS_INFO_STREAM("Attempt: Move to " << info());
}

GraspGoal::GraspGoal(const object_helper::Object &object_goal, const std::string &description)
: ObjectGoal(object_goal, description){
  goal_.goal_type = "grasp_pose";

  ROS_INFO("----");
  ROS_INFO_STREAM("Attempt: Move to " << info());
}

jaco_manipulation::PlanAndMoveArmGoal GraspGoal::goal() const {
  return ObjectGoal::goal();
}