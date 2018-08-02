#include <actionlib/client/simple_action_client.h>
#include <jaco_manipulation/PlanAndMoveArmAction.h>

int main(int argn, char* args[]) {

  ros::init(argn, args, "pam_client");

  actionlib::SimpleActionClient <jaco_manipulation::PlanAndMoveArmAction> pam_client ("plan_and_move_arm", true);

  jaco_manipulation::PlanAndMoveArmGoal goal;
  goal.goal_type = "pose";
  goal.target_pose.header.frame_id = "root";
  goal.target_pose.pose.position.x = -0.038;
  goal.target_pose.pose.position.y = -0.223;
  goal.target_pose.pose.position.z = 0.505;
  goal.target_pose.pose.orientation.x = -0.3172;
  goal.target_pose.pose.orientation.y =  0.6319;
  goal.target_pose.pose.orientation.z = 0.6319;
  goal.target_pose.pose.orientation.w = -0.3172;

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

