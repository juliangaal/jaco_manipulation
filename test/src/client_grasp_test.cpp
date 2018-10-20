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
    b.point.z = 14._cm/2.;
    b.dimensions.x = 5.5_cm;
    b.dimensions.y = 5.5_cm;
    b.dimensions.z = 14._cm;
    jmc.graspAt(b);
  }

////
  {
    jaco_manipulation::BoundingBox b;
    b.header.frame_id = "base_link";
    b.description = "box";
    b.point.x = 0.5_m;
    b.point.y = 0.3_m;
    b.point.z = 14._cm/2.;
    b.dimensions.x = 5.5_cm;
    b.dimensions.y = 5.5_cm;
    b.dimensions.z = 14._cm;
    jmc.dropAt(b);
  }

    jmc.moveTo("home");

  {
    jaco_manipulation::BoundingBox b;
    b.header.frame_id = "base_link";
    b.description = "box";
    b.point.x = 0.5_m;
    b.point.y = 0.3_m;
    b.point.z = 14._cm/2.;
    b.dimensions.x = 6.5_cm;
    b.dimensions.y = 6.5_cm;
    b.dimensions.z = 14._cm;
    jmc.graspAt(b);
  }

////
  {
    jaco_manipulation::BoundingBox b;
    b.header.frame_id = "base_link";
    b.description = "box";
    b.point.x = 0.5_m;
    b.point.y = 0.3_m;
    b.point.z = 14._cm/2.;
    b.dimensions.x = 6.5_cm;
    b.dimensions.y = 6.5_cm;
    b.dimensions.z = 14._cm;
    jmc.dropAt(b);
  }

  jmc.moveTo("home");

  {
    jaco_manipulation::BoundingBox b;
    b.header.frame_id = "base_link";
    b.description = "box";
    b.point.x = 0.5_m;
    b.point.y = 0.3_m;
    b.point.z = 14._cm/2.;
    b.dimensions.x = 7.5_cm;
    b.dimensions.y = 7.5_cm;
    b.dimensions.z = 14._cm;
    jmc.graspAt(b);
  }

////
  {
    jaco_manipulation::BoundingBox b;
    b.header.frame_id = "base_link";
    b.description = "box";
    b.point.x = 0.5_m;
    b.point.y = 0.3_m;
    b.point.z = 14._cm/2.;
    b.dimensions.x = 7.5_cm;
    b.dimensions.y = 7.5_cm;
    b.dimensions.z = 14._cm;
    jmc.dropAt(b);
  }

  jmc.moveTo("home");
//  {
//    jaco_manipulation::BoundingBox b;
//    b.header.frame_id = "base_link";
//    b.description = "box";
//    b.point.x = 0.4_m;
//    b.point.y = 0.6_m;
//    b.point.z = 6.5_cm/2.;
//    b.dimensions.x = 6.5_cm;
//    b.dimensions.y = 6.5_cm;
//    b.dimensions.z = 6.5_cm;
//    std::cout << b.dimensions.x << "x" << b.dimensions.y << std::endl;
//    jmc.graspAt(b);
//  }

//    jmc.moveTo("home");

////
//  {
//    jaco_manipulation::BoundingBox b;
//    b.header.frame_id = "base_link";
//    b.description = "box";
//    b.point.x = 0.5_m;
//    b.point.y = 0.3_m;
//    b.point.z = 6.5_cm/2.;
//    b.dimensions.x = 6.5_cm;
//    b.dimensions.y = 6.5_cm;
//    b.dimensions.z = 6.5_cm;
//    jmc.dropAt(b);
//  }

//  {
//    jaco_manipulation::BoundingBox b;
//    b.header.frame_id = "base_link";
//    b.description = "box";
//    b.point.x = 0.5_m;
//    b.point.y = 0.3_m;
//    b.point.z = 6.5_cm/2.;
//    b.dimensions.x = 7._cm;
//    b.dimensions.y = 7._cm;
//    b.dimensions.z = 6.5_cm;
//    std::cout << b.dimensions.x << "x" << b.dimensions.y << std::endl;
//    jmc.graspAt(b);
//  }
////
//  {
//    jaco_manipulation::BoundingBox b;
//    b.header.frame_id = "base_link";
//    b.description = "box";
//    b.point.x = 0.5_m;
//    b.point.y = 0.3_m;
//    b.point.z = 6.5_cm/2.;
//    b.dimensions.x = 7._cm;
//    b.dimensions.y = 7._cm;
//    b.dimensions.z = 6.5_cm;
//    jmc.dropAt(b);
//  }

//  {
//    jaco_manipulation::BoundingBox b;
//    b.header.frame_id = "base_link";
//    b.description = "box";
//    b.point.x = 0.5_m;
//    b.point.y = 0.2_m;
//    b.point.z = 20._cm + 6.5_cm/2.;
//    b.dimensions.x = 7.2_cm;
//    b.dimensions.y = 7.2_cm;
//    b.dimensions.z = 6.5_cm;
//    std::cout << b.dimensions.x << "x" << b.dimensions.y << std::endl;
//    jmc.graspAt(b);
//  }
////
//  {
//    jaco_manipulation::BoundingBox b;
//    b.header.frame_id = "base_link";
//    b.description = "box";
//    b.point.x = 0.5_m;
//    b.point.y = 0.3_m;
//    b.point.z = 6.5_cm/2.;
//    b.dimensions.x = 7.2_cm;
//    b.dimensions.y = 7.2_cm;
//    b.dimensions.z = 6.5_cm;
//    jmc.dropAt(b);
//  }
////
//  {
//    jaco_manipulation::BoundingBox b;
//    b.header.frame_id = "base_link";
//    b.description = "box";
//    b.point.x = 0.5_m;
//    b.point.y = 0.3_m;
//    b.point.z = 6.5_cm/2.;
//    b.dimensions.x = 7.6_cm;
//    b.dimensions.y = 7.6_cm;
//    b.dimensions.z = 6.5_cm;
//    std::cout << b.dimensions.x << "x" << b.dimensions.y << std::endl;
//    jmc.graspAt(b);
//  }
//
//  {
//    jaco_manipulation::BoundingBox b;
//    b.header.frame_id = "base_link";
//    b.description = "box";
//    b.point.x = 0.5_m;
//    b.point.y = 0.3_m;
//    b.point.z = 6.5_cm/2.;
//    b.dimensions.x = 7.6_cm;
//    b.dimensions.y = 7.6_cm;
//    b.dimensions.z = 6.5_cm;
//    jmc.dropAt(b);
//  }
//
//  {
//    jaco_manipulation::BoundingBox b;
//    b.header.frame_id = "base_link";
//    b.description = "box";
//    b.point.x = 0.5_m;
//    b.point.y = 0.3_m;
//    b.point.z = 6.5_cm/2.;
//    b.dimensions.x = 8._cm;
//    b.dimensions.y = 8._cm;
//    b.dimensions.z = 6.5_cm;
//    std::cout << b.dimensions.x << "x" << b.dimensions.y << std::endl;
//    jmc.graspAt(b);
//  }
//
//  {
//    jaco_manipulation::BoundingBox b;
//    b.header.frame_id = "base_link";
//    b.description = "box";
//    b.point.x = 0.5_m;
//    b.point.y = 0.3_m;
//    b.point.z = 6.5_cm/2.;
//    b.dimensions.x = 8._cm;
//    b.dimensions.y = 8._cm;
//    b.dimensions.z = 6.5_cm;
//    jmc.dropAt(b);
//  }
//
//  {
//    jaco_manipulation::BoundingBox b;
//    b.header.frame_id = "base_link";
//    b.description = "box";
//    b.point.x = 0.5_m;
//    b.point.y = 0.3_m;
//    b.point.z = 9.5_cm/2.;
//    b.dimensions.x = 10._cm;
//    b.dimensions.y = 10._cm;
//    b.dimensions.z = 9.5_cm;
//    std::cout << b.dimensions.x << "x" << b.dimensions.y << std::endl;
//    jmc.graspAt(b);
//  }
//
//  {
//    jaco_manipulation::BoundingBox b;
//    b.header.frame_id = "base_link";
//    b.description = "box";
//    b.point.x = 0.5_m;
//    b.point.y = 0.3_m;
//    b.point.z = 9.5_cm/2.;
//    b.dimensions.x = 10._cm;
//    b.dimensions.y = 10._cm;
//    b.dimensions.z = 9.5_cm;
//    jmc.dropAt(b);
//  }
////  {
////    jaco_manipulation::BoundingBox b;
////    b.header.frame_id = "base_link";
////    b.description = "box";
////    b.point.x = .7_m;
////    b.point.y = .3_m;
////    b.point.z = 14.5_cm/2.;
////    b.dimensions.x = 6._cm;
////    b.dimensions.y = 6._cm;
////    b.dimensions.z = 19.5_cm;
////    jmc.graspAt(b);
////  }
////
////  {
////    jaco_manipulation::BoundingBox b;
////    b.header.frame_id = "base_link";
////    b.description = "box";
////    b.point.x = .6_m;
////    b.point.y = .3_m;
////    b.point.z = 14.5_cm/2.;
////    b.dimensions.x = 6._cm;
////    b.dimensions.y = 6._cm;
////    b.dimensions.z = 14.5_cm;
////    jmc.dropAt(b);
////  }
////
////  {
////    jaco_manipulation::BoundingBox b;
////    b.header.frame_id = "base_link";
////    b.description = "box";
////    b.point.x = 0.5_m;
////    b.point.y = 0.5_m;
////    b.point.z =6.0_cm/2.;
////    b.dimensions.x = 6.5_cm;
////    b.dimensions.y = 6.5_cm;
////    b.dimensions.z = 6.5_cm;
////    jmc.graspAt(b);
////  }
////
////  {
////    jaco_manipulation::BoundingBox b;
////    b.header.frame_id = "base_link";
////    b.description = "box";
////    b.point.x = 0.5_m;
////    b.point.y = 0.5_m;
////    b.point.z =6.0_cm/2.;
////    b.dimensions.x = 6.5_cm;
////    b.dimensions.y = 6.5_cm;
////    b.dimensions.z = 6.5_cm;
////    jmc.dropAt(b);
////  }

  return 0;
}
