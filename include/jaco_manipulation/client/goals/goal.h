//
// Created by julian on 04.08.18.
//

#ifndef PROJECT_GOAL_H
#define PROJECT_GOAL_H

#include <jaco_manipulation/PlanAndMoveArmAction.h>
#include <string>
#include "goal.h"

using std::string;

/**
 * Goal defines the base class for a goal we can send the Jaco Robotic Arm via MoveIt.
 * That can either be a goal from the movit config (jaco.srdf) ("MoveItGoal" class),
 * a joint state goal ("JointGoal" class), or a pose goal ("PoseGoal" class)
 */
class Goal {
 public:
  /**
   * default constructor
   */
  Goal();

  /**
   * deleted constructor
   */
  Goal(const string &name) = delete;

  /**
   * virtual defaul destructor
   */
  virtual ~Goal() = default;

  /**
   * get goal that was created
   * @return jaco_manipulation::PlanAndMoveArmGoal
   */
  virtual jaco_manipulation::PlanAndMoveArmGoal getGoal() const;

 protected:
  /**
   * jaco_manipulation::PlanAndMoveArmGoal goal that will be created
   */
  jaco_manipulation::PlanAndMoveArmGoal goal;

  /**
   * planning frame for MoveIt! plans
   */
  const string planning_frame;
};

#endif //PROJECT_GOAL_H
