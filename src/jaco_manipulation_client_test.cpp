#include <actionlib/client/simple_action_client.h>
#include <jaco_manipulation/PlanAndMoveArmAction.h>

int main(int argn, char* args[]) {

  ros::init(argn, args, "pam_client");

  actionlib::SimpleActionClient <jaco_manipulation::PlanAndMoveArmAction> pam_client ("plan_and_move_arm", true);

  jaco_manipulation::PlanAndMoveArmGoal goal;
  goal.goal_type = "pose";
  goal.target_pose.header.frame_id = "root";
  goal.target_pose.pose.position.x = 0.063846;
  goal.target_pose.pose.position.y = -0.193645;
  goal.target_pose.pose.position.z = 0.509365;
  goal.target_pose.pose.orientation.x = 0.369761;
  goal.target_pose.pose.orientation.y =  -0.555344;
  goal.target_pose.pose.orientation.z = -0.661933;
  goal.target_pose.pose.orientation.w = 0.341635;

  pam_client.waitForServer();
  ROS_INFO("Calling jaco_manipulation...");
  pam_client.sendGoal(goal);
  pam_client.waitForResult();

  if(pam_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Move succeeded.");
    return 0;
  }
  else
  {
    ROS_INFO("Move failed.");
    return 0;
  }
}

