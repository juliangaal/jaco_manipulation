//
// Created by julian on 04.08.18.
//

#include <jaco_manipulation/client/goals/goal.h>

Goal::Goal() : planning_frame("root") {
  goal.goal_type = "goal";
}

jaco_manipulation::PlanAndMoveArmGoal Goal::getGoal() const {
  return goal;
}