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

#include <jaco_manipulation/goals/joint_goal.h>
#include <ros/console.h>

using namespace jaco_manipulation::goals;

JointGoal::JointGoal(const std::vector<double> &joint_goal, const std::string &description) {
  description_ = description;
  goal_.goal_type = "joint_state";
  goal_.joint_goal.position = joint_goal;
  goal_.joint_goal.header.frame_id = planning_frame_;

  ROS_INFO_STREAM("Attempt : Move to " << info());
}

jaco_manipulation::PlanAndMoveArmGoal JointGoal::goal() const {
  return Goal::goal();
}
