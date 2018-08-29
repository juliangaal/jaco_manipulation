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

#ifndef PROJECT_GOAL_INPUT_H
#define PROJECT_GOAL_INPUT_H

namespace jaco_manipulation {
namespace goals {
namespace goal_input {

/**
 * Object to grasp, defined by bounding box
 */
struct BoundingBox {
  std::string description;
  std::string type;
  double x;
  double y;
  double z;

  double height;
  double width;
  double length;
};

/**
 * additional layer of security: grasp_helper::GraspPose allows only to change properties on pose that are not dangerous
 */
struct LimitedPose {
  double x;
  double y;
  double z;
  double rotation;
};
} // goal_input
} // namespace goals
} // namespace jaco_manipulation

#endif //PROJECT_GOAL_INPUT_H
