//
// Created by julian on 04.08.18.
//

#include <jaco_manipulation/client/goals/move_it_goal.h>
#include <ros/console.h>

using namespace jaco_manipulation::client::goals;

MoveItGoal::MoveItGoal(const std::string &name, const std::string &description) {
  description_ = description;
  goal_.goal_type = name;

  ROS_INFO("----");
  ROS_INFO_STREAM("Attempt: Move to " << description_);
}

jaco_manipulation::PlanAndMoveArmGoal MoveItGoal::getGoal() const {
  return Goal::getGoal();
}

const std::string &MoveItGoal::getDescription() const {
  return Goal::getDescription();
}
