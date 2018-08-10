//
// Created by chitt bn 8/6/18.
//

#include <jaco_manipulation/client/jaco_manipulation_client.h>

using namespace jaco_manipulation;

int main(int argn, char *args[]) {

  ros::init(argn, args, "pam_client");

  client::JacoManipulationClient jmc;
//  jmc.moveTo("home");

  using namespace goals::goal_input;
  {
    BoundingBox b;
    b.description = "ball";
    b.x = 0.37;
    b.y = 0.03;
    b.length = 0.06;
    b.height = 0.06;
    b.width = 0.06;
    jmc.graspAt(b);
  }

  {
    BoundingBox b;
    b.description = "mug";
    b.x = 0.605;
    b.y = 0.045;
    b.length = 0.09;
    b.width = 0.09;
    b.height = 0.11;
    jmc.dropAt(b);
  }

  {
    LimitedPose pose;
    pose.x = 0.4;
    pose.y = 0.0;
    jmc.graspAt(pose);
    jmc.dropAt(pose);
  }

  jmc.moveTo("home");

  return 0;
}
