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

#ifndef PROJECT_MOVEITGOAL_H
#define PROJECT_MOVEITGOAL_H

#include <jaco_manipulation/goals/goal.h>

namespace jaco_manipulation {
namespace goals {

/*!
 * MoveItGoal
 * wrapper class MoveIt! Goal defined in config file
 */
class MoveItGoal : public Goal {
 public:
  /**
   * deleted default constructor
  */
  MoveItGoal() = delete;

  /**
   * constructor
   * @param goal_name name of goal_type in moveit config
   * @param description description to make client console output more clear
  */
  explicit MoveItGoal(const std::string &goal_name, const std::string &description="movit config file goal");

  /**
   * virtual default destructor
   */
  ~MoveItGoal() final = default;

  /**
    * get goal that was created
    * @return jaco_manipulation::PlanAndMoveArmGoal
  */
  jaco_manipulation::PlanAndMoveArmGoal goal() const final;
};

} // namespace goals
} // namespace jaco_manipulation

#endif //PROJECT_MOVEITGOAL_H
