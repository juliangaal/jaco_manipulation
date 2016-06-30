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
#include "jaco_manipulation/jaco_manipulation.h"

namespace jaco_manipulation
{
JacoManipulation::JacoManipulation() :
    group_("arm"),
    pam_server_(nh_, "plan_and_move_arm", boost::bind(&JacoManipulation::processGoal, this, _1), false),
    haa_client_("jaco_arm/home_arm", true)
{
  ROS_INFO("Initializing Doro Manipulation!");

  finger_pub_ = nh_.advertise<std_msgs::Float32>("jaco_arm/finger_cmd", 1);

  tf_listener_ = boost::shared_ptr<tf::TransformListener>(new tf::TransformListener(ros::Duration(10)));

  group_.setPlanningTime(10);
  pam_server_.start();
}

void JacoManipulation::processGoal(const jaco_manipulation::PlanAndMoveArmGoalConstPtr& _goal)
{
  ROS_INFO("Got a goal. Working on it...");

  bool result_value;

  if(_goal->goal_type.compare("pose") == 0)
  {
    ROS_INFO("*************");
    ROS_INFO("*Pose target*");
    ROS_INFO("*************");
    result_value = planAndMove(_goal->target_pose);
  }
  else
  {
    ROS_INFO("**************");
    ROS_INFO("*Named target*");
    ROS_INFO("**************");
    result_value = planAndMove(_goal->goal_type);
  }

  if(result_value)
  {
    ROS_INFO("Sam was a showing scalp flat top... Action succeeded.");
    pam_server_.setSucceeded();
  }

  else
  {
    ROS_INFO("For some reason the execution failed... Action aborted/failed.");
    pam_server_.setAborted();
  }
}

JacoManipulation::~JacoManipulation()
{

}

bool JacoManipulation::hasReachedPose (const geometry_msgs::PoseStamped& target_pose)
{
  geometry_msgs::PoseStamped now_pose = group_.getCurrentPose();

  if(
      (now_pose.pose.position.x - target_pose.pose.position.x < 0.01) &&
      (now_pose.pose.position.y - target_pose.pose.position.y < 0.01) &&
      (now_pose.pose.position.z - target_pose.pose.position.z < 0.01) &&
      (now_pose.pose.orientation.x - target_pose.pose.orientation.x < 0.01) &&
      (now_pose.pose.orientation.y - target_pose.pose.orientation.y < 0.01) &&
      (now_pose.pose.orientation.z - target_pose.pose.orientation.z < 0.01) &&
      (now_pose.pose.orientation.w - target_pose.pose.orientation.w < 0.01) )
  {
    ROS_INFO("Pose reached. Motion complete.");
    return true;
  }
  else
  {
    ROS_INFO("Moving...");
    return false;
  }

}

/**
 * Convenience function to plan and execute the pose specified by target_pose
 */
bool JacoManipulation::planAndMove(const geometry_msgs::PoseStamped& target_pose)
{

  group_.allowReplanning(true);
  group_.allowLooking(true);
  group_.setStartStateToCurrentState();

  ROS_INFO("FRAME FOR PLANNING:= %s",group_.getPoseReferenceFrame().c_str());

  group_.setPoseReferenceFrame (target_pose.header.frame_id);
  group_.setPoseTarget(target_pose);
  //group_.setRandomTarget();

  geometry_msgs::PoseStamped the_pose_now = group_.getCurrentPose("jaco_link_hand");

  ROS_INFO("The pose now: (%f,%f,%f) ; (%f,%f,%f,%f)",
      the_pose_now.pose.position.x,
      the_pose_now.pose.position.y,
      the_pose_now.pose.position.z,
      the_pose_now.pose.orientation.x,
      the_pose_now.pose.orientation.y,
      the_pose_now.pose.orientation.z,
      the_pose_now.pose.orientation.w);
  ROS_INFO("FRAME FOR CURRENT POSE:= %s",the_pose_now.header.frame_id.c_str());

  ROS_INFO("The target pose: (%f,%f,%f) ; (%f,%f,%f,%f)",
      target_pose.pose.position.x,
      target_pose.pose.position.y,
      target_pose.pose.position.z,
      target_pose.pose.orientation.x,
      target_pose.pose.orientation.y,
      target_pose.pose.orientation.z,
      target_pose.pose.orientation.w);
  ROS_INFO("FRAME FOR TARGET POSE:= %s",target_pose.header.frame_id.c_str());

  bool success = group_.plan(plan_);

  if(success)
    ROS_INFO("PLAN FOUND!");
  else
  {
    ROS_INFO("A plan was not found.");
    return false;
  }

  if(group_.move())
  {
    ROS_INFO("Motion complete.");
    return true;
  }
  else
  {
    ROS_INFO("Oh! Sucks. Life sucks...");
    return false;
  }

}

bool JacoManipulation::planAndMove(const std::string& target_pose_string)
{

  if(target_pose_string == "open" || target_pose_string == "OPEN") {
    moveGripper(0.0);
    sleep(3.0);
    return true;
  }
  else if(target_pose_string == "close" || target_pose_string == "CLOSE") {
    moveGripper(6500.00);
    sleep(3.0);
    return true;
  }
  else if(target_pose_string == "home" || target_pose_string == "HOME") {
    haa_client_.waitForServer();
    ROS_INFO("[JacoManipulation]: Homing Arm...");
    wpi_jaco_msgs::HomeArmGoal hag;
    hag.retract = false;
    haa_client_.sendGoal(hag);
    haa_client_.waitForResult();

    if(haa_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      return true;
    }
    else return false;
  }

  else {

    group_.allowReplanning(true);
    group_.allowLooking(true);
    group_.setStartStateToCurrentState();

    ROS_INFO("FRAME FOR PLANNING:= %s",group_.getPoseReferenceFrame().c_str());

    group_.setNamedTarget(target_pose_string);
    //group_.setRandomTarget();

    bool success = group_.plan(plan_);

    if(success)
      ROS_INFO("PLAN FOUND!");
    else
    {
      ROS_INFO("A plan was not found.");
      return false;
    }

    if(group_.move())
    {
      ROS_INFO("Motion complete.");
      return true;
    }
    else
    {
      ROS_INFO("Oh! Sucks. Life sucks...");
      return false;
    }

  }

}
/**
 * Convenience function to plan the pose specified by target_pose
 */
bool JacoManipulation::plan(const geometry_msgs::PoseStamped& target_pose)
{

  //group_.setStartStateToCurrentState();

  group_.setPoseReferenceFrame (target_pose.header.frame_id);
  group_.setPoseTarget(target_pose);
  geometry_msgs::PoseStamped the_pose_now = group_.getCurrentPose();

  ROS_INFO("The pose now: (%f,%f,%f) ; (%f,%f,%f,%f)",
      the_pose_now.pose.position.x,
      the_pose_now.pose.position.y,
      the_pose_now.pose.position.z,
      the_pose_now.pose.orientation.x,
      the_pose_now.pose.orientation.y,
      the_pose_now.pose.orientation.z,
      the_pose_now.pose.orientation.w);

  ROS_INFO("The target pose: (%f,%f,%f) ; (%f,%f,%f,%f)",
      target_pose.pose.position.x,
      target_pose.pose.position.y,
      target_pose.pose.position.z,
      target_pose.pose.orientation.x,
      target_pose.pose.orientation.y,
      target_pose.pose.orientation.z,
      target_pose.pose.orientation.w);

  return group_.plan(plan_);
}


/**
 * Attaches the table obstacle to the planning scene interface.
 */
void JacoManipulation::addTableAsObstacle(geometry_msgs::PoseStamped table_pose)
{
  ros::param::set("/plane_extraction_enable", true);

  while(table_pose.pose.position.x == table_pose.pose.position.y &&
      table_pose.pose.position.x == table_pose.pose.position.z &&
      table_pose.pose.position.x == 0)
  {
    ROS_INFO("Waiting for table position...");
    sleep(1);
  }

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = group_.getPlanningFrame();

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

  ps_interface_.addCollisionObjects(collision_objects);


  ROS_INFO("THE PLANE WAS ADDED! SEE RVIZ!");
  //std::cout<<"Table eqn:= "<<table_coeffs<<std::endl;
  //std::cout<<"Table position:= "<<table_position<<std::endl;

  ros::param::set("/plane_extraction_enable", false);
}

/**
 * Attaches the object that is going to be picked up as obstacle.
 */
void JacoManipulation::addTargetAsObstacle(geometry_msgs::PoseStamped box_pose)
{
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = group_.getPlanningFrame();

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

  ps_interface_.addCollisionObjects(collision_objects);

  ROS_INFO("The box was added as a collision object!");
}

void JacoManipulation::removeTable()
{
  std::vector<std::string> object_ids;
  object_ids.push_back("table");
  ps_interface_.removeCollisionObjects(object_ids);
}

void JacoManipulation::removeTarget()
{
  std::vector<std::string> object_ids;
  object_ids.push_back("table");
  ps_interface_.removeCollisionObjects(object_ids);
}

void JacoManipulation::attachTarget()
{
  group_.attachObject("pill_box");
  ROS_INFO("Pill box is in gripper now.");
}

void JacoManipulation::detachTarget()
{
  group_.detachObject("pill_box");
  removeTarget();
  ROS_INFO("Pill box removed.");
}

void JacoManipulation::moveGripper(float value)
{
  std_msgs::Float32 FP;
  FP.data = value;
  finger_pub_.publish(FP);
}

}
