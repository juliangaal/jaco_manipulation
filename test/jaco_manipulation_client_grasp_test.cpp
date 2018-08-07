//
// Created by chitt on 8/6/18.
//

#include <jaco_manipulation/client/jaco_manipulation_client.h>

using namespace jaco_manipulation::client;

int main(int argn, char *args[]) {

  ros::init(argn, args, "pam_client");

  JacoManipulationClient jmc;
  jmc.moveTo("home");

  using namespace goals::object_helper;

  {
    LimitedPose pose;
    pose.x = 0.0;
    pose.y = -0.4;
    pose.rotation = 0.674663;
    jmc.grasp(pose);
  }

  {
    LimitedPose pose;
    pose.x = 0.0;
    pose.y = -0.65;
    pose.z = 0.35;
    pose.rotation = 0.674663;
    jmc.drop(pose);
  }

  jmc.moveTo("home");

  return 0;
}
