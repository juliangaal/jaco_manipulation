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

#include <jaco_manipulation/goals/goal.h>
#include <ros/console.h>

using namespace jaco_manipulation::goals;

Goal::Goal() : planning_frame_("root") {
  goal_.goal_type = "goal";
  description_ = goal_.goal_type;
}

jaco_manipulation::PlanAndMoveArmGoal Goal::goal() const {
  return goal_;
}

const std::string Goal::info() const {
  return description_ + " [goal_type=\"" + goal_.goal_type + "\"]";
}
