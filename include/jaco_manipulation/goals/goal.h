/*
  Copyright (C) 2018  Julian Gaal
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#ifndef PROJECT_GOAL_H
#define PROJECT_GOAL_H

#include <jaco_manipulation/PlanAndMoveArmAction.h>
#include <string>
#include "goal.h"

namespace jaco_manipulation {
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
} // namespace jaco_manipulation

#endif //PROJECT_GOAL_H
