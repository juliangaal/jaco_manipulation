//
// Created by julian on 04.08.18.
//

#ifndef PROJECT_JOINTGOAL_H
#define PROJECT_JOINTGOAL_H

#include <vector>
#include "goal.h"

using std::vector;

/**
 * JointGoal
 * class to define a joint state goal for MoveIt! execution
 */
class JointGoal : public Goal {
 public:
  /**
   * deleted default constructor
   */
  JointGoal() = delete;

  /**
   * deleted constructor
   * @param name future goal_type
   */
  JointGoal(const string &name) = delete;

  /**
   * constructor
   * @param joint_goal joint goal that will be tried to move to
   */
  explicit JointGoal(const vector<double> &joint_goal);

  /**
   * virtual default destructor
   */
  virtual ~JointGoal() = default;

  /**
 * get goal that was created
 * @return jaco_manipulation::PlanAndMoveArmGoal
 */
  virtual jaco_manipulation::PlanAndMoveArmGoal getGoal() const final;
};

#endif //PROJECT_JOINTGOAL_H
