//
// Created by chitt on 8/6/18.
//

#include <jaco_manipulation/client/jaco_manipulation_client.h>

using namespace jaco_manipulation::client;

int main(int argn, char *args[]) {

  ros::init(argn, args, "pam_client");

  JacoManipulationClient jmc;
  jmc.moveTo("home");

  {
    goals::GraspGoal::GraspPose pose;
    pose.x = 0.0;
    pose.y = -0.4;
    pose.rotation = 0.674663;
    jmc.moveTo(pose);
  }

  {
    goals::GraspGoal::GraspPose pose;
    pose.x = 0.03;
    pose.y = -0.60;
    pose.rotation = 0.674663;
    jmc.moveTo(pose);
  }

  jmc.moveTo("home");

  return 0;
}
