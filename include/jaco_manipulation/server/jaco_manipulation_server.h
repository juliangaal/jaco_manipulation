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
#include <jaco_manipulation/visuals/moveit_visuals.h>
#include <jaco_manipulation/JacoDebug.h>
#include <wpi_jaco_msgs/HomeArmAction.h>

#include <string>
#include <vector>

#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"

#define ROS_STATUS(x) ROS_INFO_STREAM("\033[34m" << x << "\033[00m")
#define ROS_SUCCESS(x) ROS_INFO_STREAM("\033[32m" << x << "\033[00m")

namespace jaco_manipulation {
namespace server {

/**
 * Convenience class to talk to Moveit-ROS interface.
*/
class JacoManipulationServer {
 public:

  /**
   * default constructor
  */
  JacoManipulationServer();

  /**
   * default destructor
  */
  ~JacoManipulationServer() = default;

  /**
   * Callback for the action server.
  */
  void processGoal(const jaco_manipulation::PlanAndMoveArmGoalConstPtr &goal);

 private:

  /**
   * A common NodeHandle.
  */
  ros::NodeHandle nh_;

  /**
   * The move_group variable.
  */
  moveit::planning_interface::MoveGroupInterface move_group_;

  /**
   * Class that handles visuals in the planning scene
   */
  jaco_manipulation::visuals::MoveitVisuals moveit_visuals_;

  /**
   * Whether or not to lock jaco in a cage
   */
  bool enable_cage_;

  /**
   * Action client to home home the arm.
  */
  actionlib::SimpleActionClient<wpi_jaco_msgs::HomeArmAction> haa_client_;

  /**
   * Action server that is used for manipulation.
  */
  actionlib::SimpleActionServer<jaco_manipulation::PlanAndMoveArmAction> pam_server_;

  tf::TransformListener tf_listener_;

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
   * A publisher to get debug information
   */
  ros::Publisher debug_pub_;

  /**
   * A message with dug information that is published at /jaco_manipulation/debug
   */
  jaco_manipulation::JacoDebug debug_msg_;

  /**
   * True if object is gripped. This needs to be handled server side due to
   * a bug in MoveIt!, where the attached object is not correctly tracked while the robot is moving
   */
  bool has_gripped_;

  /**
   * Fallback for ROS paramter llow_replanning
   */
  bool allow_replanning_;

  /**
   * Fallback for ROS paramter allow_looking
   */
  bool allow_looking_;

  /**
   * Fallback for ROS paramter planning_time
   */
  float planning_time_;

  /**
   * Fallback for ROS paramter planning_attempts
   */
  int planning_attempts_;

  /**
   * Fallback for ROS paramter planner_id
   */
  std::string planner_id_;

  /**
   * Publishes to /jaco_manipulation/debug if set true from launch file
   */
  bool publish_debug_;

  /**
  * A function to prepare MoveIt movegroup and cofigure it for all future plans
 */
  void prepMoveItMoveGroup();

  /**
   * A function to visualize planned move in RViz
  */
  void showPlannedPath();

  /**
   * Show planned move info in console from start pose to end pose
   * @param start start pose
   * @param end target pose
  */
  void showPlannedMoveInfo(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &end);

  /**
   * Show planned move info in console from start joint to end joint state
   * @param start start values
   * @param end target pose
  */
  void showPlannedMoveInfo(const std::vector<double> &start, const sensor_msgs::JointState &end);

  /**
   * Show planned move info in console from start joint to end joint state
   * @param start start pose
   * @param end target pose defined in moveit config
 */
  void showPlannedMoveInfo(const geometry_msgs::PoseStamped &start, const std::string &target);

  /**
   * Convenience function to plan and execute the pose specified by pose_goal.
   * @param pose_goal pose to be planned for and moved to
  */
  bool planAndMove(const geometry_msgs::PoseStamped &pose_goal);

  /**
   * Convenience function to plan, execute the pose and grasp for an object specified by pose_goal.
   * @param goal to move to and grasp
  */
  bool planAndMoveAndGrasp(const jaco_manipulation::PlanAndMoveArmGoalConstPtr &goal);

  /**
   * Convenience function to plan, execute the post grasp lift up of object
   */
  bool planAndMovePostGrasp();

  /**
   * Convenience function to plan, execute the pose and drop an object specified by pose_goal.
   * @param goal to move to and drop
  */
  bool planAndMoveAndDrop(const jaco_manipulation::PlanAndMoveArmGoalConstPtr &goal);

  /**
   * Convenience function to plan and execute the joint_state specified by target_joint_state.
   * @param target_joint_state joint state to move to
  */
  bool planAndMove(const sensor_msgs::JointState &target_joint_state);

  /**
   * Convenience function to plan and execute the pose specified by string (move it config name)
   * @param pose_goal_string name of string defined in moveit config, that describes pose goal
  */
  bool planAndMove(const std::string &pose_goal_string);

  /**
   * A function to close jaco's grippers
  */
  void closeGripper();

  /**
   * A function to close jaco's grippers according to size of the bounding box
   * @param box
   */
  void closeGripper(const jaco_manipulation::BoundingBox &box);

  /**
   * A function to close jaco's grippers
  */
  void openGripper();

  /**
   * A function to move jaco's grippers.
   * @param value gripper value to move to
  */
  void moveGripper(float value = 6500.0);

  /**
   * Adds obstacle into moveit planning scene
   * @param goal that includes pose and bounding box
   */
  void addObstacle(const jaco_manipulation::PlanAndMoveArmGoalConstPtr &goal);

  /**
   * Attach gripped obstacle
   * @param goal from which to get bounding box to attach
   */
  void attachObstacle(const jaco_manipulation::PlanAndMoveArmGoalConstPtr &goal);

  /**
   * Detach dropped obstacle
   * @param goal from which to get bounding box to drop
   */
  void detachObstacle(const jaco_manipulation::PlanAndMoveArmGoalConstPtr &goal);

  /**
   * Remove obstacle from planning scene
   * @param goal from which to get Bounding box to remove
   */
  void removeObstacle(const jaco_manipulation::PlanAndMoveArmGoalConstPtr &goal);

  /**
   * Wipes obstacles added by kinect from the planning scene
   */
  void wipeKinectObstacles();

  /**
   * Fills MoveItConfigMsg with values from current MoveIt! config
   * debug_msg message to be filled
   */
  void fillMoveItConfigMsg(jaco_manipulation::MoveItConfig &config_msg);

  /**
    * Fills MoveItGoalMsg with values from current MoveIt! state
    * debug_msg message to be filled
    */
  void fillMoveItGoalMsg(jaco_manipulation::MoveItGoal &goal_msg, const PlanAndMoveArmGoalConstPtr &goal);

  /**
   * Publishes debug message
   */
  void pubDebugMsg(const PlanAndMoveArmGoalConstPtr &goal, bool result);

  /// helper: joint 1
  constexpr static size_t JOINT1 = 0;

  /// helper: joint 2
  constexpr static size_t JOINT2 = 1;

  /// helper: joint 3
  constexpr static size_t JOINT3 = 2;

  /// helper: joint 4
  constexpr static size_t JOINT4 = 3;

  //// helper: joint 5
  constexpr static size_t JOINT5 = 4;

  //// helper: joint 6
  constexpr static size_t JOINT6 = 5;
};

} // namespace server
} // namespace jaco_manipulation

#endif /* JACO_MANIPULATION_H_*/
