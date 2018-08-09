//
// Created by chitt bn 8/6/18.
//

#include <jaco_manipulation/client/jaco_manipulation_client.h>

using namespace jaco_manipulation::client;

int main(int argn, char *args[]) {

  ros::init(argn, args, "pam_client");

  JacoManipulationClient jmc;
//  jmc.moveTo("home");

  using namespace goals::kinect_goal_definitions;
  {
    LimitedPose pose;
    pose.x = 0.4;
    pose.y = 0.0;
    pose.rotation = 0.674663;
    jmc.graspAt(pose);
  }
  {
    BoundingBox b;
    b.description = "mug";
    b.x = 0.605;
    b.y = 0.0;
    b.length = 0.09;
    b.width = 0.09;
    b.height = 0.11;
    jmc.dropAt(b);
  }
  {
    BoundingBox b;
    b.description = "mug";
    b.x = 0.605;
    b.y = -0.45;
    b.length = 0.09;
    b.width = 0.09;
    b.height = 0.11;
    jmc.dropAt(b);
  }
//  geometry_msgs::PoseStamped pose;
//  pose.pose.position.x = 0.063846;
//  pose.pose.position.y = -0.193645;
//  pose.pose.position.z = 0.509365;
//  pose.pose.orientation.x = 0.9765;
//  pose.pose.orientation.y = -0.155;
//  pose.pose.orientation.z = 0.023;
//  pose.pose.orientation.w = 0.153;
//  jmc.moveTo(pose);

  jmc.moveTo("home");

  return 0;
}
