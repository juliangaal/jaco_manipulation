/*
  Copyright (C) 2015  Chittaranjan Srinivas Swaminathan

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
#include <jaco_manipulation/server/jaco_manipulation.h>

using moveit::planning_interface::MoveItErrorCode;
namespace rvt = rviz_visual_tools;

namespace jaco_manipulation {

JacoManipulation::JacoManipulation() :
    move_group_("arm"),
    pam_server_(nh_, "plan_and_move_arm", boost::bind(&JacoManipulation::processGoal, this, _1), false),
    visual_tools_("root"),
    haa_client_("jaco_arm/home_arm", true) {
  ROS_INFO("Initializing Jaco Manipulation!");

  finger_pub_ = nh_.advertise<std_msgs::Float32>("jaco_arm/finger_cmd", 1);

  tf_listener_ = boost::shared_ptr<tf::TransformListener>(new tf::TransformListener(ros::Duration(10)));

  move_group_.setPlanningTime(0.5);
  move_group_.setPlannerId("RRTstarkConfigDefault");

  pam_server_.start();

  prepMoveItVisualTools();
}

void JacoManipulation::prepMoveItVisualTools() {
  visual_tools_.deleteAllMarkers();
  visual_tools_.loadRemoteControl();
  visual_tools_.trigger();
}

void JacoManipulation::showPlannedPath() {
  visual_tools_.deleteAllMarkers();
  visual_tools_.publishTrajectoryLine(plan_.trajectory_, move_group_.getCurrentState()->getJointModelGroup("arm"));
  visual_tools_.trigger();
}

void JacoManipulation::processGoal(const jaco_manipulation::PlanAndMoveArmGoalConstPtr &goal) {
  bool result_value;

  if (goal->goal_type.empty()) {
    ROS_ERROR("Goal not set. Returning");
    result_value = false;
  } else if (goal->goal_type == "pose") {
    ROS_STATUS("Goal received: pose");
    result_value = planAndMove(goal->pose_goal);
  } else if (goal->goal_type == "joint_state") {
    ROS_STATUS("Goal received: joint state");
    result_value = planAndMove(goal->joint_goal);
  } else {
    ROS_STATUS("Goal received: named target");
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

bool JacoManipulation::planAndMove(const PoseStamped &pose_goal) {
  move_group_.allowReplanning(true);
  move_group_.allowLooking(true);
  move_group_.setStartStateToCurrentState();
  move_group_.setPoseReferenceFrame(pose_goal.header.frame_id);
  move_group_.setPoseTarget(pose_goal);

  const auto &current_pose = move_group_.getCurrentPose("jaco_link_hand");

  if (move_group_.plan(plan_) != MoveItErrorCode::SUCCESS) return false;

  showPlannedMoveInfo(current_pose, pose_goal);
  showPlannedPath();

  return move_group_.move() ? true : false;
}

bool JacoManipulation::planAndMove(const JointState &joint_goal) {
  // TODO BUG CAN"T USE SENSOR_MSGS/JOINTSTATE, MAYBE BECAUSE OLY POSITION IS FILLED
  move_group_.allowReplanning(true);
  move_group_.allowLooking(true);
  move_group_.setStartStateToCurrentState();
  move_group_.setPoseReferenceFrame(joint_goal.header.frame_id);
  moveit::core::RobotStatePtr current_state = move_group_.getCurrentState();

  std::vector<double> joint_group_positions;
  joint_group_positions.push_back(joint_goal.position[0]);
  joint_group_positions.push_back(joint_goal.position[1]);
  joint_group_positions.push_back(joint_goal.position[2]);
  joint_group_positions.push_back(joint_goal.position[3]);
  joint_group_positions.push_back(joint_goal.position[4]);
  joint_group_positions.push_back(joint_goal.position[5]);

  move_group_.setJointValueTarget(joint_group_positions);

  const auto &current_joint_state = move_group_.getCurrentJointValues();

  if (move_group_.plan(plan_) != MoveItErrorCode::SUCCESS) return false;

  showPlannedMoveInfo(current_joint_state, joint_goal);
  showPlannedPath();

  return move_group_.move() ? true : false;
}

bool JacoManipulation::planAndMove(const std::string &pose_goal_string) {

  if (pose_goal_string == "open" || pose_goal_string == "OPEN") {
    moveGripper(0.0);
    sleep(3.0);
    return true;
  } else if (pose_goal_string == "close" || pose_goal_string == "CLOSE") {
    moveGripper(6500.00);
    sleep(3.0);
    return true;
  } else {
    move_group_.allowReplanning(true);
    move_group_.allowLooking(true);
    move_group_.setStartStateToCurrentState();
    move_group_.setNamedTarget(pose_goal_string);

    ROS_INFO_STREAM("FRAME FOR PLANNING := " << move_group_.getPoseReferenceFrame());

    if (move_group_.plan(plan_) != MoveItErrorCode::SUCCESS) return false;

    showPlannedPath();

    return move_group_.move() ? true : false;
  }
}

/**
 * Attaches the table obstacle to the planning scene interface.
 */
void JacoManipulation::addTableAsObstacle(PoseStamped table_pose) {
  ros::param::set("/plane_extraction_enable", true);

  while (table_pose.pose.position.x == table_pose.pose.position.y &&
      table_pose.pose.position.x == table_pose.pose.position.z &&
      table_pose.pose.position.x == 0) {
    ROS_INFO("Waiting for table position...");
    sleep(1);
  }

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_.getPlanningFrame();

  /* The id of the object is used to identify it. */
  collision_object.id = "table";

  /* Define a box to add to the world. */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.8;
  primitive.dimensions[1] = 0.8;
  primitive.dimensions[2] = 0.025;

  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::Pose table_pose_;
  table_pose_.orientation.w = 1.0;
  table_pose_.position.x = table_pose.pose.position.x;
  table_pose_.position.y = table_pose.pose.position.y;
  table_pose_.position.z = table_pose.pose.position.z;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(table_pose_);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  planning_scene_interface_.addCollisionObjects(collision_objects);

  ROS_INFO("THE PLANE WAS ADDED! SEE RVIZ!");

  ros::param::set("/plane_extraction_enable", false);
}

/**
 * Attaches the object that is going to be picked up as obstacle.
 */
void JacoManipulation::addTargetAsObstacle(PoseStamped box_pose) {
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_.getPlanningFrame();

  /* The id of the object is used to identify it. */
  collision_object.id = "pill_box";

  /* Define a box to add to the world. */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.025;
  primitive.dimensions[1] = 0.035;
  primitive.dimensions[2] = 0.080;

  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::Pose pill_box_pose;
  pill_box_pose.orientation.w = 1.0;
  pill_box_pose.position.x = box_pose.pose.position.x;
  pill_box_pose.position.y = box_pose.pose.position.y;
  pill_box_pose.position.z = box_pose.pose.position.z;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(pill_box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  planning_scene_interface_.addCollisionObjects(collision_objects);

  ROS_INFO("The box was added as a collision object!");
}

void JacoManipulation::removeTable() {
  std::vector<std::string> object_ids;
  object_ids.emplace_back("table");
  planning_scene_interface_.removeCollisionObjects(object_ids);
}

void JacoManipulation::removeTarget() {
  std::vector<std::string> object_ids;
  object_ids.emplace_back("table");
  planning_scene_interface_.removeCollisionObjects(object_ids);
}

void JacoManipulation::attachTarget() {
  move_group_.attachObject("pill_box");
  ROS_INFO("Pill box is in gripper now.");
}

void JacoManipulation::detachTarget() {
  move_group_.detachObject("pill_box");
  removeTarget();
  ROS_INFO("Pill box removed.");
}

void JacoManipulation::moveGripper(float value) {
  std_msgs::Float32 FP;
  FP.data = value;
  finger_pub_.publish(FP);
}

void JacoManipulation::addBoundaries() {

}

void JacoManipulation::showPlannedMoveInfo(const PoseStamped &start,
                                           const PoseStamped &end) {
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

void JacoManipulation::showPlannedMoveInfo(const vector<double> &start, const JointState &end) {
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
}
