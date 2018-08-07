//
// Created by julian on 06.08.18.
//

#include <jaco_manipulation/client/goals/objects/drop_goal.h>
#include <ros/console.h>

using namespace jaco_manipulation::client::goals::objects;

DropGoal::DropGoal(const object_helper::LimitedPose &drop_pose_goal, const std::string &description)
: ObjectGoal(drop_pose_goal, description) {
  goal_.goal_type = "drop_pose";

  ROS_INFO("----");
  ROS_INFO_STREAM("Attempt: Move to " << info());
}

DropGoal::DropGoal(const object_helper::Object &object_goal, const std::string &description)
: ObjectGoal(object_goal, description) {
  goal_.goal_type = "drop_pose";

  ROS_INFO("----");
  ROS_INFO_STREAM("Attempt: Move to " << info());
}

jaco_manipulation::PlanAndMoveArmGoal DropGoal::goal() const {
  return ObjectGoal::goal();
}
