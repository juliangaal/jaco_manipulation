#include <actionlib/client/simple_action_client.h>
#include <jaco_manipulation/PlanAndMoveArmAction.h>

int main(int argn, char* args[]) {

  ros::init(argn, args, "pam_client");

  actionlib::SimpleActionClient <jaco_manipulation::PlanAndMoveArmAction> pam_client ("plan_and_move_arm", true);
  
  jaco_manipulation::PlanAndMoveArmGoal _goal;
  _goal.goal_type = "pose";
  _goal.target_pose.header.frame_id = "root";
  _goal.target_pose.pose.position.x = -0.038;
  _goal.target_pose.pose.position.y = -0.223;
  _goal.target_pose.pose.position.z = 0.505;
  _goal.target_pose.pose.orientation.x = -0.3172;
  _goal.target_pose.pose.orientation.y =  0.6319;
  _goal.target_pose.pose.orientation.z = 0.6319;
  _goal.target_pose.pose.orientation.w = -0.3172;
  
  pam_client.waitForServer();
  ROS_INFO("Calling jaco_manipulation...");
  pam_client.sendGoal(_goal);
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

