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

#ifndef PROJECT_POSEGOAL_H
#define PROJECT_POSEGOAL_H

#include <jaco_manipulation/goals/goal.h>

namespace jaco_manipulation {
namespace goals {

/*!
 * PoseGoal
 * class to define a pose goal for MoveIt! execution
*/
class PoseGoal : public Goal {
 public:

  /**
   * deleted constructor
   * @param name future goal_type
   */
  PoseGoal(const std::string &name) = delete;

  /**
   * constructor
   * @param pose geometry_msgs::PoseStamped to move to
   * @param description description to make client console output more clear
   */
  explicit PoseGoal(const geometry_msgs::PoseStamped &pose, const std::string &description="pose goal");

  /**
   * virtual default destructor
   */
  virtual ~PoseGoal() override = default;

  /**
   * get goal that was created
   * @return jaco_manipulation::PlanAndMoveArmGoal
 */
  virtual jaco_manipulation::PlanAndMoveArmGoal goal() const override;

 protected:

  /**
   * protected default constructor
  */
  PoseGoal() = default;

  /**
   * Adjusts height for poses that are too low
  */
  void adjustHeight();

  /**
   * Minimum height for pose
  */
  constexpr static double min_height_ = 0.175026;
};

} // namespace goals
} // namespace jaco_manipulation

#endif //PROJECT_POSEGOAL_H
