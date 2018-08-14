//
// Created by chitt bn 8/6/18.
//

#include <jaco_manipulation/client/jaco_manipulation_client.h>
#include <jaco_manipulation/BoundingBox.h>

using namespace jaco_manipulation;

int main(int argn, char *args[]) {

  ros::init(argn, args, "pam_client");

  client::JacoManipulationClient jmc;
  jmc.moveTo("home");

  {
    jaco_manipulation::BoundingBox b;
    b.header.frame_id = "base_link";
    b.description = "bottle";
    b.point.x = 0.30;
    b.point.y = 0.25;
    b.dimensions.x = 0.05;
    b.dimensions.y = 0.05;
    b.dimensions.z = 0.21;
    jmc.graspAt(b);
  }

  {
    jaco_manipulation::BoundingBox b;
    b.header.frame_id = "base_link";
    b.description = "bottle";
    b.point.x = 0.50;
    b.point.y = 0.25;
    b.dimensions.x = 0.05;
    b.dimensions.y = 0.05;
    b.dimensions.z = 0.21;
    jmc.dropAt(b);
  }

//  jmc.moveTo("home");

  return 0;
}
