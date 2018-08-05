//
// Created by julian on 04.08.18.
//

#ifndef PROJECT_POSEGOAL_H
#define PROJECT_POSEGOAL_H

#include "goal.h"

class PoseGoal : public Goal {
 public:
  PoseGoal() = delete;
  PoseGoal(const string &name) = delete;
  explicit PoseGoal(const geometry_msgs::PoseStamped &pose);
  virtual ~PoseGoal() = default;

  virtual jaco_manipulation::PlanAndMoveArmGoal getGoal() const final;
};

#endif //PROJECT_POSEGOAL_H
