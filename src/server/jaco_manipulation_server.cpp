/*
  Copyright (C) 2018  Julian Gaal
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#include <jaco_manipulation/server/jaco_manipulation_server.h>
#include <thread>
#include <chrono>

using moveit::planning_interface::MoveItErrorCode;
using namespace jaco_manipulation::server;
namespace rvt = rviz_visual_tools;

JacoManipulationServer::JacoManipulationServer() :
    move_group_("arm"),
    pam_server_(nh_, "plan_and_move_arm", boost::bind(&JacoManipulationServer::processGoal, this, _1), false),
    plan_{},
    haa_client_("jaco_arm/home_arm", true),
    moveit_visuals_(nh_, "root", move_group_, plan_),
    tf_listener_(nh_, ros::Duration(10)) {
  ROS_INFO("Initializing Jaco Manipulation!");

  finger_pub_ = nh_.advertise<std_msgs::Float32>("jaco_arm/finger_cmd", 1);

  pam_server_.start();

  prepMoveItMoveGroup();

  sleep(3);

  openGripper();
}

void JacoManipulationServer::prepMoveItMoveGroup() {
  move_group_.setPlanningTime(1.0);
//  move_group_.setPlannerId("RRTConnectkConfigDefault");
  move_group_.setPlannerId("RRTstarkConfigDefault");
  move_group_.setNumPlanningAttempts(5);
  move_group_.allowReplanning(true);
  move_group_.allowLooking(true);
}

void JacoManipulationServer::showPlannedPath() {
  moveit_visuals_.showPlannedPath();
}

void JacoManipulationServer::processGoal(const jaco_manipulation::PlanAndMoveArmGoalConstPtr &goal) {
  bool result_value;

  if (goal->goal_type.empty() || goal->goal_type == "goal") {
    ROS_ERROR("Goal not set or configured correctly. Returning");
    result_value = false;
  } else if (goal->goal_type == "pose") {
    result_value = planAndMove(goal->pose_goal);
  } else if (goal->goal_type == "joint_state") {
    result_value = planAndMove(goal->joint_goal);
  } else if (goal->goal_type == "grasp_pose") {
    result_value = planAndMoveAndGrasp(goal);
  } else if (goal->goal_type == "drop_pose") {
    result_value = planAndMoveAndDrop(goal);
  } else if (goal->goal_type == "add_obstacle") {
    addObstacle(goal);
  } else if (goal->goal_type == "remove_obstacle") {
    removeObstacle(goal);
  } else {
    result_value = planAndMove(goal->goal_type);
  }

  if (result_value) {
    ROS_SUCCESS("Plan found. Action succeeded.");
    pam_server_.setSucceeded();
  } else {
    ROS_ERROR("NO Plan found. Action aborted/failed.");
    pam_server_.setAborted();
  }
}

bool JacoManipulationServer::planAndMove(const geometry_msgs::PoseStamped &pose_goal) {
  ROS_STATUS("Goal received: pose");

  move_group_.setStartStateToCurrentState();
  move_group_.setPoseReferenceFrame(pose_goal.header.frame_id);
  move_group_.setPoseTarget(pose_goal);

  showPlannedMoveInfo(move_group_.getCurrentPose("jaco_link_hand"), pose_goal);

  if (move_group_.plan(plan_) != MoveItErrorCode::SUCCESS) return false;
  else showPlannedPath();

  return move_group_.move() ? true : false;
}

bool JacoManipulationServer::planAndMove(const sensor_msgs::JointState &joint_goal) {
  // TODO BUG CAN"T USE SENSOR_MSGS/JOINTSTATE, MAYBE BECAUSE OLY POSITION IS FILLED
  ROS_STATUS("Goal received: joint state");

  move_group_.setStartStateToCurrentState();
  move_group_.setPoseReferenceFrame(joint_goal.header.frame_id);
  move_group_.setJointValueTarget(joint_goal.position);

  showPlannedMoveInfo(move_group_.getCurrentJointValues(), joint_goal);

  if (move_group_.plan(plan_) != MoveItErrorCode::SUCCESS) return false;
  else showPlannedPath();

  return move_group_.move() ? true : false;
}

bool JacoManipulationServer::planAndMove(const std::string &pose_goal_string) {
  ROS_STATUS("Goal received: named target");

  move_group_.setStartStateToCurrentState();
  move_group_.setNamedTarget(pose_goal_string);

  showPlannedMoveInfo(move_group_.getCurrentPose("jaco_link_hand"), pose_goal_string);

  if (move_group_.plan(plan_) != MoveItErrorCode::SUCCESS) return false;
  else showPlannedPath();

  return move_group_.move() ? true : false;
}

bool JacoManipulationServer::planAndMoveAndGrasp(const jaco_manipulation::PlanAndMoveArmGoalConstPtr &goal) {
  ROS_STATUS("Grasp request received");

  addObstacle(goal);

  bool moved = planAndMove(goal->pose_goal);
  if (!moved) return false;
//  closeGripper(goal->bounding_box);

  attachObstacle(goal);
//  closeGripper(goal->bounding_box);
  closeGripper();
  ROS_STATUS("Gripper closed. Object grasped.");

  // once gripped we simply move up a little
  jaco_manipulation::PlanAndMoveArmGoal new_goal(*goal);
  new_goal.pose_goal.pose.position.z += 0.2;

  moved = planAndMove(new_goal.pose_goal);
  if (!moved) return false;

  ROS_STATUS("Moved to pose gripping state");

  return true;
}

bool JacoManipulationServer::planAndMoveAndDrop(const jaco_manipulation::PlanAndMoveArmGoalConstPtr &goal) {
  ROS_STATUS("Drop request received");

  bool moved = planAndMove(goal->pose_goal);
  if (!moved) {
    openGripper();
    detachObstacle(goal);
    return false;
  }

  openGripper();
  detachObstacle(goal);

  ROS_STATUS("Gripper opened. Object dropped.");

  return true;
}

void JacoManipulationServer::addObstacle(const jaco_manipulation::PlanAndMoveArmGoalConstPtr &goal) {
  moveit_visuals_.addObstacle(goal);
}

void JacoManipulationServer::attachObstacle(const jaco_manipulation::PlanAndMoveArmGoalConstPtr &goal) {
  moveit_visuals_.attachObstacle(goal);
}

void JacoManipulationServer::detachObstacle(const jaco_manipulation::PlanAndMoveArmGoalConstPtr &goal) {
  moveit_visuals_.detachObstacle(goal);
}

void JacoManipulationServer::removeObstacle(const jaco_manipulation::PlanAndMoveArmGoalConstPtr &goal) {
  moveit_visuals_.removeObstacle(goal->bounding_box.description);
}

void JacoManipulationServer::moveGripper(float value) {
  std_msgs::Float32 FP;
  FP.data = value;
  finger_pub_.publish(FP);
  sleep(3);
}

void JacoManipulationServer::closeGripper() {
  moveGripper(6500.0);
}

void JacoManipulationServer::closeGripper(const jaco_manipulation::BoundingBox &box) {
  // define gripper close by a function y = mx + b. In our case x is size of bounding box and y is amount to grip
  // our function is: y = -433.33blabla + 6500
  constexpr double max_grip = 6500.0;
  const double size = std::max(box.dimensions.x, box.dimensions.y);
  float amount = -43333.333 * size + 6500.0;

  // give it a tiny extra squeeze, for heavier objects e.g.
  constexpr float squeeze = 350.0;
  amount += squeeze;

  if (amount < 0.0) {
    ROS_ERROR("Object is too big to be gripped!");
    amount = 0.0;
  }

  moveGripper(amount);
}

void JacoManipulationServer::openGripper() {
  moveGripper(0.0);
}

void JacoManipulationServer::showPlannedMoveInfo(const geometry_msgs::PoseStamped &start,
                                           const geometry_msgs::PoseStamped &end) {
  ROS_INFO_STREAM("Frame for Planning := " << move_group_.getPoseReferenceFrame());
  ROS_INFO("The pose now: (%f,%f,%f) ; (%f,%f,%f,%f)",
           start.pose.position.x,
           start.pose.position.y,
           start.pose.position.z,
           start.pose.orientation.x,
           start.pose.orientation.y,
           start.pose.orientation.z,
           start.pose.orientation.w);
  ROS_INFO_STREAM("Frame for current Pose := " << start.header.frame_id);
  ROS_INFO("The target pose: (%f,%f,%f) ; (%f,%f,%f,%f)",
           end.pose.position.x,
           end.pose.position.y,
           end.pose.position.z,
           end.pose.orientation.x,
           end.pose.orientation.y,
           end.pose.orientation.z,
           end.pose.orientation.w);
  ROS_INFO_STREAM("FRAME FOR TARGET POSE := " << end.header.frame_id);
}

void JacoManipulationServer::showPlannedMoveInfo(const std::vector<double> &start, const sensor_msgs::JointState &end) {
  assert(start.size() >= 6);
  ROS_INFO_STREAM("Frame for Planning := " << move_group_.getPoseReferenceFrame());
  ROS_INFO("The joint states now: (%f,%f,%f,%f,%f,%f)",
           start[JOINT1],
           start[JOINT2],
           start[JOINT3],
           start[JOINT4],
           start[JOINT5],
           start[JOINT6]);
  ROS_INFO("The target joint states: (%f,%f,%f,%f,%f,%f)",
           end.position[JOINT1],
           end.position[JOINT2],
           end.position[JOINT3],
           end.position[JOINT4],
           end.position[JOINT5],
           end.position[JOINT6]);
  ROS_INFO_STREAM("FRAME FOR TARGET POSE := " << end.header.frame_id);
}

void JacoManipulationServer::showPlannedMoveInfo(const geometry_msgs::PoseStamped &start, const std::string &target) {
  ROS_INFO_STREAM("Frame for Planning := " << move_group_.getPoseReferenceFrame());
  ROS_INFO("The pose now: (%f,%f,%f) ; (%f,%f,%f,%f)",
           start.pose.position.x,
           start.pose.position.y,
           start.pose.position.z,
           start.pose.orientation.x,
           start.pose.orientation.y,
           start.pose.orientation.z,
           start.pose.orientation.w);
  ROS_INFO_STREAM("Frame for current Pose := " << start.header.frame_id);
  ROS_INFO_STREAM("The moveit config target pose: " << target);
}
