//
// Created by julian on 06.08.18.
//

#include <jaco_manipulation/client/goals/objects/drop_goal.h>
#include <ros/console.h>

using namespace jaco_manipulation::client::goals::objects;

DropGoal::DropGoal(const grasp_helper::GraspPose &drop_pose_goal, const std::string &description)
: ObjectGoal(drop_pose_goal, description) {
  goal_.goal_type = "drop_pose";
}

DropGoal::DropGoal(const grasp_helper::Object &object_goal, const std::string &description)
: ObjectGoal(object_goal, description) {
  goal_.goal_type = "drop_pose";
}

jaco_manipulation::PlanAndMoveArmGoal DropGoal::getGoal() const {
  return ObjectGoal::getGoal();
}

const std::string &DropGoal::getDescription() const {
  return ObjectGoal::getDescription();
}
