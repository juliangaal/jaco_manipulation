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

#ifndef PROJECT_GRASPGOAL_HPP
#define PROJECT_GRASPGOAL_HPP
#include "vision_goal.h"

namespace jaco_manipulation {
namespace goals {

/*!
 * GraspGoal
 * Represent a severely limited pose for grasping objects
 * Limited in height, and rotation around x and y axis
 */
class GraspGoal: public VisionGoal {
 public:

  /**
   * deleted default constructor
  */
  GraspGoal() = delete;

  /**
   * Constructor
   * @param grasp_pose_goal grasp pose goal
   * @param description descritpion with additional info
   */
  explicit GraspGoal(const goal_input::LimitedPose &grasp_pose_goal,
                     jaco_manipulation::grasps::GraspType grasp,
                     const std::string &description = "grasp goal");

  /**
   * Constructor
   * @param bounding_box_goal bounding box to drop something at
   * @param description descritpion with additional info
   */
  explicit GraspGoal(const jaco_manipulation::BoundingBox&bounding_box_goal,
                     jaco_manipulation::grasps::GraspType grasp,
                     const std::string &description = "grasp box goal");

  /**
   * default destructor
   */
  ~GraspGoal() final = default;

  /**
   * get created goal
   */
  jaco_manipulation::PlanAndMoveArmGoal goal() const final;
};

} // namespace goals
} // namespace jaco_manipulation

#endif //PROJECT_GRASPGOAL_HPP
