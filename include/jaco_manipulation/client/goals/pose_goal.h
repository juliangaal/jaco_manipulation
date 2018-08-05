//
// Created by julian on 04.08.18.
//

#ifndef PROJECT_POSEGOAL_H
#define PROJECT_POSEGOAL_H

#include "goal.h"

/**
 * PoseGoal
 * class to define a pose goal for MoveIt! execution
 */
class PoseGoal : public Goal {
 public:
  /**
   * deleted default constructor
   */
  PoseGoal() = delete;

  /**
   * deleted constructor
   * @param name future goal_type
   */
  PoseGoal(const string &name) = delete;

  /**
   * constructor
   * @param pose geometry_msgs::PoseStamped to move to
   */
  explicit PoseGoal(const geometry_msgs::PoseStamped &pose);

  /**
   * virtual default destructor
   */
  virtual ~PoseGoal() = default;

  /**
   * get goal that was created
   * @return jaco_manipulation::PlanAndMoveArmGoal
 */
  virtual jaco_manipulation::PlanAndMoveArmGoal getGoal() const final;
};

#endif //PROJECT_POSEGOAL_H
