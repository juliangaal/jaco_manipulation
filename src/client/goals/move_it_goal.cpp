//
// Created by julian on 04.08.18.
//

#include <jaco_manipulation/client/goals/move_it_goal.h>

MoveItGoal::MoveItGoal(const string &name) {
  goal.goal_type = name;
}

jaco_manipulation::PlanAndMoveArmGoal MoveItGoal::getGoal() const {
  return goal;
}