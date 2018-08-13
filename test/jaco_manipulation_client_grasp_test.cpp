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

  using namespace goals::goal_input;
//  {
//    BoundingBox b;
//    b.description = "ball";
//    b.x = 0.37;
//    b.y = 0.03;
//    b.length = 0.06;
//    b.height = 0.06;
//    b.width = 0.06;
//    jmc.graspAt(b);
//  }

  {
    jaco_manipulation::BoundingBox b;
    b.header.frame_id = "base_link";
    b.description = "bottle";
    b.point.x = 0.65;
    b.point.y = 0.272;
    b.dimensions.x = 0.09;
    b.dimensions.y = 0.09;
    b.dimensions.z = 0.21;
    jmc.graspAt(b);
  }

  {
    jaco_manipulation::BoundingBox b;
    b.header.frame_id = "base_link";
    b.description = "bottle";
    b.point.x = 0.44;
    b.point.y = 0.272;
    b.dimensions.x = 0.09;
    b.dimensions.y = 0.09;
    b.dimensions.z = 0.21;
    jmc.dropAt(b);
  }

  jmc.moveTo("home");

  return 0;
}
