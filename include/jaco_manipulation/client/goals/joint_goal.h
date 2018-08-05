//
// Created by julian on 04.08.18.
//

#ifndef PROJECT_JOINTGOAL_H
#define PROJECT_JOINTGOAL_H

#include "move_it_goal.h"
#include <vector>

using std::vector;

class JointGoal : public MoveItGoal {
  JointGoal() = delete;
  JointGoal(const string &name);
  JointGoal(const string &name, const vector<double> &joint_goal);
  virtual ~JointGoal() = default;

  virtual jaco_manipulation::PlanAndMoveArmGoal getGoal() const final;

  void setGoal(const vector<double> &joint_goal);
};

#endif //PROJECT_JOINTGOAL_H
