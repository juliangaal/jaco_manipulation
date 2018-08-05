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
#ifndef JACO_MANIPULATION_H_
#define JACO_MANIPULATION_H_

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <jaco_manipulation/PlanAndMoveArmAction.h>
#include <jaco_manipulation/grasp_pose_generator.h>
#include <wpi_jaco_msgs/HomeArmAction.h>

#include <string>
#include <vector>

#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"

#define ROS_STATUS(x) ROS_INFO_STREAM("\033[34m" << (x) << "\033[00m")
#define ROS_SUCCESS(x) ROS_INFO_STREAM("\033[32m" << (x) << "\033[00m")

using geometry_msgs::Pose;
using geometry_msgs::PoseStamped;
using sensor_msgs::JointState;
using std::string;
using std::vector;

namespace jaco_manipulation {
/**
 * Convenience class to talk to Moveit-ROS interface.
 */
class JacoManipulation {
 private:
  /**
   * MoveIt visual tools
   */
  moveit_visual_tools::MoveItVisualTools visual_tools_;

  /**
   * A common NodeHandle.
   */
  ros::NodeHandle nh_;

  /**
   * Action client to home home the arm.
   */
  actionlib::SimpleActionClient<wpi_jaco_msgs::HomeArmAction> haa_client_;

  /**
   * Action server that is used for manipulation.
   */
  actionlib::SimpleActionServer<jaco_manipulation::PlanAndMoveArmAction> pam_server_;

  /**
   * The plan variable.
   */
  moveit::planning_interface::MoveGroupInterface::Plan plan_;

  /**
   * The planning scene interface.
   * This we use to add obstacles. These obstacles are the planes.
   */
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  /**
   * A publisher to control jaco's hand grip.
   */
  ros::Publisher finger_pub_;

  /**
   * Convenience variable to get the current pose of jaco.
   * This is not updated by a callback.
   */
  geometry_msgs::PoseStamped current_pose_;

  boost::shared_ptr<tf::TransformListener> tf_listener_;

  /**
   * Callback for the action server.
   */
  void processGoal(const jaco_manipulation::PlanAndMoveArmGoalConstPtr &_goal);

 public:

  /// helper: joint 1
  constexpr static size_t JOINT1 = 0;

  /// helper: joint 1
  constexpr static size_t JOINT2 = 1;

  // helper: joint 1
  constexpr static size_t JOINT3 = 2;

  // helper: joint 1
  constexpr static size_t JOINT4 = 3;

  // helper: joint 1
  constexpr static size_t JOINT5 = 4;

  // helper: joint 1
  constexpr static size_t JOINT6 = 5;

  /**
   * The move_group variable.
   */
  moveit::planning_interface::MoveGroupInterface move_group_;

  /**
   * default constructor
   */
  JacoManipulation();

  /*
   * default destructor
   */
  ~JacoManipulation() = default;

  /**
   * A function to prepare MoveIt! Visual Tools in RViz
   */
  void prepMoveItVisualTools();

  /**
   * A function to visualize planned move in RViz
   */
  void showPlannedPath();

  /**
   * Show planned move info in console from start pose to end pose
   */
  void showPlannedMoveInfo(const PoseStamped &start, const PoseStamped &end);

  /**
   * Show planned move info in console from start joint to end joint state
   */
  void showPlannedMoveInfo(const vector<double> &start, const JointState &end);

  /**
  * Show planned move info in console from start joint to end joint state
  */
  void showPlannedMoveInfo(const PoseStamped &start, const string &target);

  /**
   * A function to add boundaries (for workspace in Oerebro, for now)
   */
  void addBoundaries();

  /**
   * A function to add the table as an obstacle.
   */
  void addTableAsObstacle(PoseStamped table_pose);

  /**
   * A function to add the target as an obstacle.
   */
  void addTargetAsObstacle(PoseStamped box_pose);

  /**
   * Remove the Table after task is complete.
   */
  void removeTable();

  /**
   * Remove the Target after release.
   */
  void removeTarget();

  // Sucky
  void attachTarget();

  // Sucky 2
  void detachTarget();

  /**
   * Convenience function to plan and execute the pose specified by target_pose.
   */
  bool planAndMove(const PoseStamped &target_pose);

  /**
   * Convenience function to plan and execute the joint_state specified by target_joint_state.
   */
  bool planAndMove(const JointState &target_joint_state);
  /**
   * Convenience function to plan and execute the pose specified by string.
   */
  bool planAndMove(const std::string &target_pose_string);

  /**
   * Convenience function to plan the pose specified by target_pose.
   */
  bool plan(const PoseStamped &target_pose);

  /**
   * A function to move doro's grippers.
   */
  void moveGripper(float value = 6500.0);

  /**
   * Reset the values after a run.
   */
  void resetValues();

};
}
#endif /* JACO_MANIPULATION_H_ */
