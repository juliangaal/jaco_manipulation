//
// Created by chitt bn 8/6/18.
//

#include <jaco_manipulation/client/jaco_manipulation_client.h>

using namespace jaco_manipulation::client;

int main(int argn, char *args[]) {

  ros::init(argn, args, "pam_client");

  JacoManipulationClient jmc;
  jmc.moveTo("home");

  using namespace goals::kinect_goal;

  {
    LimitedPose pose;
    pose.x = 0.0;
    pose.y = -0.4;
    pose.rotation = 0.674663;
    jmc.graspAt(pose);
  }

  {
    BoundingBox b;
    b.description = "mug";
    b.x = 0.045;
    b.y = -0.695;
    b.length = 0.09;
    b.width = 0.09;
    b.height = 0.11;
    jmc.dropAt(b);
  }

  jmc.moveTo("home");

  return 0;
}
