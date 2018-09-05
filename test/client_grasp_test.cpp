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
#include <jaco_manipulation/units.h>

using namespace jaco_manipulation;

int main(int argn, char *args[]) {

  ros::init(argn, args, "pam_client");

  client::JacoManipulationClient jmc;

  {
    jaco_manipulation::BoundingBox b;
    b.header.frame_id = "base_link";
    b.description = "box";
    b.point.x = 0.5_m;
    b.point.y = 0.3_m;
    b.point.z = 2.05_cm;
    b.dimensions.x = 4.1_cm;
    b.dimensions.y = 4.1_cm;
    b.dimensions.z = 4.1_cm;
    jmc.graspAt(b);
  }

  {
    jaco_manipulation::BoundingBox b;
    b.header.frame_id = "base_link";
    b.description = "box";
    b.point.x = 0.4_m;
    b.point.y = 0.3_m;
    b.point.z = 2.05_cm;
    b.dimensions.x = 4.1_cm;
    b.dimensions.y = 4.1_cm;
    b.dimensions.z = 4.1_cm;
    jmc.dropAt(b);
    jmc.graspAt(b);
  }

  {
    jaco_manipulation::BoundingBox b;
    b.header.frame_id = "base_link";
    b.description = "box";
    b.point.x = 0.5_m;
    b.point.y = 0.3_m;
    b.point.z = 2.05_cm;
    b.dimensions.x = 4.1_cm;
    b.dimensions.y = 4.1_cm;
    b.dimensions.z = 4.1_cm;
    jmc.dropAt(b);
  }

  return 0;
}
