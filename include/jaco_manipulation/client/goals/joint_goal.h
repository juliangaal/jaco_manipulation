//
// Created by julian on 04.08.18.
//

#ifndef PROJECT_JOINTGOAL_H
#define PROJECT_JOINTGOAL_H

#include <vector>
#include "goal.h"

namespace jaco_manipulation {
namespace client {
namespace goals {

/*!
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
  JointGoal(const std::string &name) = delete;

  /**
   * constructor
   * @param joint_goal joint goal that will be tried to move to
   * @param description description to make client console output more clear
   */
  explicit JointGoal(const std::vector<double> &joint_goal, const std::string &description = "joint goal");

  /**
   * virtual default destructor
   */
   ~JointGoal() final = default;

  /**
   * get goal that was created
   * @return jaco_manipulation::PlanAndMoveArmGoal
  */
  jaco_manipulation::PlanAndMoveArmGoal goal() const final;
};

} // namespace goals
} // namespace client
} // namespace jaco_manipulation

#endif //PROJECT_JOINTGOAL_H
