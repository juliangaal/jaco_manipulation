//
// Created by julian on 04.08.18.
//

#ifndef PROJECT_GOAL_H
#define PROJECT_GOAL_H

#include <jaco_manipulation/PlanAndMoveArmAction.h>
#include <string>
#include "goal.h"

namespace jaco_manipulation {
namespace client {
namespace goals {

/*!
 * Goal defines the base class for a goal we can send the Jaco Robotic Arm via MoveIt.
 * That can either be a goal from the movit config (jaco.srdf) ("MoveItGoal" class),
 * a joint state goal ("JointGoal" class), or a pose goal ("PoseGoal" class)
 */
class Goal {
 public:

  /**
   * deleted constructor
   */
  Goal(const std::string &name) = delete;

  /**
   * virtual defaul destructor
   */
  virtual ~Goal() = default;

  /**
   * get goal that was created
   * @return jaco_manipulation::PlanAndMoveArmGoal
   */
  virtual jaco_manipulation::PlanAndMoveArmGoal goal() const;

  /**
   * get description of goal
   * @return std::string description
  */
  virtual const std::string info() const;

 protected:

  /**
   * default constructor
  */
  Goal();

  /**
   * jaco_manipulation::PlanAndMoveArmGoal goal that will be created
   */
  jaco_manipulation::PlanAndMoveArmGoal goal_;

  /**
   * planning frame for MoveIt! plans
   */
  const std::string planning_frame_;

  /**
   * description to make client output more clear
  */
  std::string description_;
};

} // namespace goals
} // namespace client
} // namespace jaco_manipulation

#endif //PROJECT_GOAL_H
