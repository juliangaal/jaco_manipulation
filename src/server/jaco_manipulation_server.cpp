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
#include <jaco_manipulation/units.h>
#include <thread>
#include <chrono>
#include <ctime>

using moveit::planning_interface::MoveItErrorCode;
using namespace jaco_manipulation::server;
namespace rvt = rviz_visual_tools;

JacoManipulationServer::JacoManipulationServer() :
    move_group_("arm"),
    pam_server_(nh_, "plan_and_move_arm", boost::bind(&JacoManipulationServer::processGoal, this, _1), false),
    plan_{},
    haa_client_("jaco_arm/home_arm", true),
    moveit_visuals_(nh_, "root", move_group_, plan_),
    tf_listener_(nh_, ros::Duration(10)),
    has_gripped_(false),
    allow_replanning_(false),
    allow_looking_(false),
    planning_time_(10.),
    planning_attempts_(1),
    planner_id_("RRTConnectkConfigDefault"),
    publish_debug_(false)
{
  ROS_INFO("Initializing Jaco Manipulation!");

  finger_pub_ = nh_.advertise<std_msgs::Float32>("jaco_arm/finger_cmd", 1);
  debug_pub_  = nh_.advertise<jaco_manipulation::JacoDebug>("jaco_manipulation/debug", 1);

  pam_server_.start();

  prepMoveItMoveGroup();

  openGripper();
}

void JacoManipulationServer::prepMoveItMoveGroup() {
  ROS_INFO_STREAM("Setting ROS Parameters from launch file");

  if (nh_.getParam("jaco_manipulation_server/planner_id", planner_id_)) {
    ROS_INFO_STREAM("Got param 'planner_id': " << planner_id_);
  } else {
    ROS_ERROR_STREAM("Failed to get param 'planner_id'");
  }

  if (nh_.getParam("jaco_manipulation_server/planning_time", planning_time_)) {
    ROS_INFO_STREAM("Got param 'planning_time': " << planning_time_);
  } else {
    ROS_ERROR_STREAM("Failed to get param 'planning_time'");
  }

  if (nh_.getParam("jaco_manipulation_server/planning_attempts", planning_attempts_)) {
    ROS_INFO_STREAM("Got param 'planning_attempts': " << planning_attempts_);
  } else {
    ROS_ERROR_STREAM("Failed to get param 'planning_attempts'");
  }

  if (nh_.getParam("jaco_manipulation_server/allow_replanning", allow_replanning_)) {
    ROS_INFO_STREAM("Got param 'allow_replanning': " << allow_replanning_);
  } else {
    ROS_ERROR_STREAM("Failed to get param 'allow_replanning'");
  }

  if (nh_.getParam("jaco_manipulation_server/allow_looking", allow_looking_)) {
    ROS_INFO_STREAM("Got param 'allow_looking': " << allow_looking_);
  } else {
    ROS_ERROR_STREAM("Failed to get param 'allow_looking'");
  }

  if (nh_.getParam("jaco_manipulation_server/publish_debug", publish_debug_)) {
    ROS_INFO_STREAM("Got param 'publish_debug': " << (publish_debug_ ? "True" : "False"));
  } else {
    ROS_ERROR_STREAM("Failed to get param 'publish_debug'");
  }

  move_group_.setPlannerId(planner_id_);
  move_group_.setPlanningTime(planning_time_);
  move_group_.setNumPlanningAttempts(planning_attempts_);
  move_group_.allowReplanning(allow_replanning_);
  move_group_.allowLooking(allow_looking_);
}

void JacoManipulationServer::showPlannedPath() {
  moveit_visuals_.showPlannedPath();
}

void JacoManipulationServer::processGoal(const jaco_manipulation::PlanAndMoveArmGoalConstPtr &goal) {
  static bool result_value    = false;
  static bool obstacle_action = false;

  if (goal->goal_type.empty() || goal->goal_type == "goal") {
    ROS_ERROR("Goal not set or configured correctly. Returning");
    result_value = false;
  } else if (goal->goal_type == "pose") {
    result_value = planAndMove(goal->pose_goal);
  } else if (goal->goal_type == "joint_state") {
    result_value = planAndMove(goal->joint_goal);
  } else if (goal->goal_type == "grasp_pose") {
    result_value = planAndMoveAndGrasp(goal);
    pubDebugMsg(goal, result_value);
  } else if (goal->goal_type == "drop_pose") {
    result_value = planAndMoveAndDrop(goal);
    pubDebugMsg(goal, result_value);
  } else if (goal->goal_type == "add_obstacle") {
    addObstacle(goal);
    pam_server_.setSucceeded();
    return;
  } else if (goal->goal_type == "wipe_kinect_obstacles") {
    wipeKinectObstacles();
    pam_server_.setSucceeded();
    return;
  } else if (goal->goal_type == "post_grasp") {
    result_value = planAndMovePostGrasp();
    pubDebugMsg(goal, result_value);
  } else if (goal->goal_type == "remove_obstacle") {
    removeObstacle(goal);
    pam_server_.setSucceeded();
    return;
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
  if (!moved) {
    return false;
  }

  attachObstacle(goal);
  closeGripper(goal->bounding_box);
  has_gripped_ = true;
  ROS_STATUS("Gripper closed. Object grasped.");

  return true;
}

bool JacoManipulationServer::planAndMovePostGrasp() {
  ROS_STATUS("Goal received: post grasp pose");

  if (!has_gripped_) {
    ROS_WARN_STREAM("Not moving to pose grasp pose: notthin gripped");
    return true;
  }

  auto pose_goal = move_group_.getCurrentPose();
  pose_goal.pose.position.z += 13._cm;

  return planAndMove(pose_goal);
}

bool JacoManipulationServer::planAndMoveAndDrop(const jaco_manipulation::PlanAndMoveArmGoalConstPtr &goal) {
  ROS_STATUS("Drop request received");

  if (!has_gripped_) {
    ROS_WARN_STREAM("Not moving to drop pose: notthin gripped. Moving home");
    return planAndMove("home");
  }

  bool moved = planAndMove(goal->pose_goal);
  if (!moved) {
    openGripper();
    detachObstacle(goal);
    return false;
  }

  openGripper();
  detachObstacle(goal);
  has_gripped_ = false;
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

void JacoManipulationServer::wipeKinectObstacles() {
  moveit_visuals_.wipeKinectObstacles();
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
  const double size = std::min(box.dimensions.x, box.dimensions.y);
  auto amount = static_cast<float>(-43333.333 * size + 6500.0);

  // give it a tiny extra squeeze, for heavier objects e.g.
  constexpr float squeeze = 1000.;
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

void JacoManipulationServer::fillMoveItConfigMsg(jaco_manipulation::MoveItConfig &config_msg) {
  config_msg.planner = planner_id_;
  config_msg.planning_time = planning_time_;
  config_msg.planning_attempts = planning_attempts_;
  config_msg.allow_looking = allow_looking_;
  config_msg.allow_replanning = allow_replanning_;
}

void JacoManipulationServer::fillMoveItGoalMsg(jaco_manipulation::MoveItGoal &goal_msg,
                                               const jaco_manipulation::PlanAndMoveArmGoalConstPtr &goal) {
  goal_msg.current_pose = move_group_.getCurrentPose();
  geometry_msgs::PointStamped out_pt;
  geometry_msgs::PointStamped in_pt;
  in_pt.header = goal->pose_goal.header;
  in_pt.header.frame_id = move_group_.getPlanningFrame();
  in_pt.point = goal->pose_goal.pose.position;

  // transform point into base link for logging
  try {
    tf_listener_.waitForTransform("base_link", move_group_.getPlanningFrame(), in_pt.header.stamp, ros::Duration(2));
    tf_listener_.transformPoint("base_link", in_pt, out_pt);
  }
  catch (tf::TransformException &exception) {
    ROS_INFO_STREAM("Debug Pub: Transform from base_link to " << move_group_.getPlanningFrame() << " failed. Why? - " << exception.what());
  }
  geometry_msgs::PoseStamped pose;
  pose.pose.position = out_pt.point;
  pose.pose.orientation = goal->pose_goal.pose.orientation;
  goal_msg.target_pose = pose;
  goal_msg.bounding_box = goal->bounding_box;
}

void JacoManipulationServer::pubDebugMsg(const jaco_manipulation::PlanAndMoveArmGoalConstPtr &goal, bool result) {
  if (!publish_debug_) return;

  auto sysTimePoint = std::chrono::system_clock::now();
  auto tp = std::chrono::system_clock::to_time_t(sysTimePoint);
  debug_msg_.timestamp = std::asctime(std::gmtime(&tp));
  debug_msg_.goal.description = goal->goal_type;
  debug_msg_.planning_scene.num_of_obstacles = moveit_visuals_.numOfObstacles();
  debug_msg_.planning_scene.obstacles = moveit_visuals_.getObstacles();
  fillMoveItConfigMsg(debug_msg_.config);
  fillMoveItGoalMsg(debug_msg_.goal, goal);
  debug_msg_.result = (result == true) ? "success" : "failure";
  debug_pub_.publish(debug_msg_);
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
