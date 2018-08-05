//
// Created by julian on 04.08.18.
//

#ifndef PROJECT_JOINTGOAL_H
#define PROJECT_JOINTGOAL_H

#include <vector>
#include "goal.h"

using std::vector;

class JointGoal : public Goal {
 public:
  JointGoal() = delete;
  JointGoal(const string &name) = delete;
  explicit JointGoal(const vector<double> &joint_goal);
  virtual ~JointGoal() = default;

  virtual jaco_manipulation::PlanAndMoveArmGoal getGoal() const final;

  void setGoal(const vector<double> &joint_goal);
};

#endif //PROJECT_JOINTGOAL_H
