//
// Created by julian on 04.08.18.
//

#include "../../include/jaco_manipulation/client/goals/move_it_goal.h"

MoveItGoal::MoveItGoal() : planning_frame("root") {
  goal.goal_type = "home";
}

MoveItGoal::MoveItGoal(const string &name) : planning_frame("root") {
  goal.goal_type = name;
}

jaco_manipulation::PlanAndMoveArmGoal MoveItGoal::getGoal() const {
  return goal;
}