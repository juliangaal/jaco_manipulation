#include <jaco_manipulation/client/jaco_manipulation_client.h>

using namespace jaco_manipulation::client;

int main(int argn, char *args[]) {

  ros::init(argn, args, "pam_client");

  JacoManipulationClient jmc;

  jmc.moveTo("home");

  sensor_msgs::JointState joint_state;
  joint_state.position.push_back(-2.6435937802859897);
  joint_state.position.push_back(2.478897506888874);
  joint_state.position.push_back(1.680057969995632);
  joint_state.position.push_back(-2.0813597278055846);
  joint_state.position.push_back(1.451960752633381);
  joint_state.position.push_back(1.0931317536782839);
  jmc.moveTo(joint_state);

  jmc.moveTo("joint_state_2");

  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0.063846;
  pose.pose.position.y = -0.193645;
  pose.pose.position.z = 0.509365;
  pose.pose.orientation.x = 0.369761;
  pose.pose.orientation.y = -0.555344;
  pose.pose.orientation.z = -0.661933;
  pose.pose.orientation.w = 0.341635;
  jmc.moveTo(pose);

  jmc.moveTo("home");

  return 0;
}

