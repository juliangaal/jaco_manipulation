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

#include <jaco_manipulation/client/jaco_manipulation_client.h>
#include <jaco_manipulation/BoundingBox.h>

using namespace jaco_manipulation;

int main(int argn, char *args[]) {

  ros::init(argn, args, "pam_client");

  client::JacoManipulationClient jmc;

  {
    jaco_manipulation::BoundingBox b;
    b.header.frame_id = "base_link";
    b.description = "ball";
    b.point.x = 0.4;
    b.point.y = 0.3;
    b.point.z = 0.03;
    b.dimensions.x = 0.06;
    b.dimensions.y = 0.06;
    b.dimensions.z = 0.06;
    jmc.graspAt(b);
  }

  {
    jaco_manipulation::BoundingBox b;
    b.header.frame_id = "base_link";
    b.description = "ball";
    b.point.x = 0.4;
    b.point.y = 0.1;
    b.point.z = 0.03;
    b.dimensions.x = 0.06;
    b.dimensions.y = 0.06;
    b.dimensions.z = 0.06;
    jmc.dropAt(b);
  }

//  {
//    jaco_manipulation::BoundingBox b;
//    b.header.frame_id = "base_link";
//    b.description = "cup";
//    b.point.x = 0.4;
//    b.point.y = 0.1;
//    b.point.z = 0.05;
//    b.dimensions.x = 0.06;
//    b.dimensions.y = 0.06;
//    b.dimensions.z = 0.10;
//    jmc.graspAt(b);
//  }
//
//
//  {
//    jaco_manipulation::BoundingBox b;
//    b.header.frame_id = "base_link";
//    b.description = "cup";
//    b.point.x = 0.4;
//    b.point.y = 0.3;
//    b.point.z = 0.05;
//    b.dimensions.x = 0.06;
//    b.dimensions.y = 0.06;
//    b.dimensions.z = 0.10;
//    jmc.dropAt(b);
//  }

//  jmc.moveTo("home");

//  {
//    jaco_manipulation::BoundingBox b;
//    b.header.frame_id = "base_link";
//    b.description = "bottle";
//    b.point.x = 0.695;
//    b.point.y = 0.3;
//    b.point.z = 0.105;
//    b.dimensions.x = 0.06;
//    b.dimensions.y = 0.06;
//    b.dimensions.z = 0.21;
//    jmc.graspAt(b);
//  }
//
//  {
//    jaco_manipulation::BoundingBox b;
//    b.header.frame_id = "base_link";
//    b.description = "bottle";
//    b.point.x = 0.55;
//    b.point.y = 0.0;
//    b.point.z = 0.105;
//    b.dimensions.x = 0.06;
//    b.dimensions.y = 0.06;
//    b.dimensions.z = 0.21;
//    jmc.dropAt(b);
//  }
//  jmc.moveTo("home");

  return 0;
}
