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

#include <jaco_manipulation/goals/grasp_goal.h>
#include <ros/console.h>

using namespace jaco_manipulation::goals;

GraspGoal::GraspGoal(const goal_input::LimitedPose &grasp_pose_goal,
                     jaco_manipulation::grasps::GraspType grasp,
                     const std::string &description)

: VisionGoal(grasp_pose_goal, grasp, description) {
  goal_.goal_type = "grasp_pose";

  ROS_INFO_STREAM("Attempt : Move to " << info());
}

GraspGoal::GraspGoal(const jaco_manipulation::BoundingBox &bounding_box_goal,
                     jaco_manipulation::grasps::GraspType grasp,
                     const std::string &description)
: VisionGoal(bounding_box_goal, grasp, description) {
  goal_.goal_type = "grasp_pose";

  ROS_INFO_STREAM("Attempt : Move to " << info());
}

jaco_manipulation::PlanAndMoveArmGoal GraspGoal::goal() const {
  return VisionGoal::goal();
}
