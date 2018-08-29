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

#ifndef PROJECT_JOINTGOAL_H
#define PROJECT_JOINTGOAL_H

#include <vector>
#include "goal.h"

namespace jaco_manipulation {
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
} // namespace jaco_manipulation

#endif //PROJECT_JOINTGOAL_H
