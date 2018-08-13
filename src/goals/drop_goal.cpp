//
// Created by julian on 06.08.18.
//

#include <jaco_manipulation/goals/drop_goal.h>
#include <ros/console.h>

using namespace jaco_manipulation::goals;

DropGoal::DropGoal(const goal_input::LimitedPose &drop_pose_goal, const std::string &description)
: KinectGoal(drop_pose_goal, jaco_manipulation::grasps::GraspType::TOP_DROP, description) {
  goal_.goal_type = "drop_pose";

  ROS_INFO_STREAM("Attempt : Move to " << info());
}

DropGoal::DropGoal(goal_input::BoundingBox &bounding_box_goal, const std::string &description)
: KinectGoal(bounding_box_goal, jaco_manipulation::grasps::GraspType::TOP_DROP, description) {
  goal_.goal_type = "drop_pose";

  ROS_INFO_STREAM("Attempt : Move to " << info());
}

jaco_manipulation::PlanAndMoveArmGoal DropGoal::goal() const {
  return KinectGoal::goal();
}

