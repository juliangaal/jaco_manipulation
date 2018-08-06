//
// Created by julian on 04.08.18.
//

#include <jaco_manipulation/client/goals/goal.h>
#include <ros/console.h>

using namespace jaco_manipulation::client::goals;

Goal::Goal() : planning_frame_("root") {
  goal_.goal_type = "goal";
  description_ = goal_.goal_type;
}

jaco_manipulation::PlanAndMoveArmGoal Goal::getGoal() const {
  return goal_;
}

const std::string &Goal::getDescription() const {
  return description_;
}
