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

#ifndef PROJECT_MOVEIT_VISUALS_H
#define PROJECT_MOVEIT_VISUALS_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <jaco_manipulation/BoundingBox.h>
#include <jaco_manipulation/PlanAndMoveArmAction.h>

namespace jaco_manipulation {
namespace visuals {

/**
 * A class that handles all the interaction with planning scene in rviz
 */
class MoveitVisuals {
 public:
  MoveitVisuals() = delete;

  /**
   * constructor
   * @param nh nodehandle
   * @param frame planning frame
   * @param move_group move group interface
   * @param plan move it plan interface
   */
  MoveitVisuals(ros::NodeHandle &nh, const std::string frame,
                moveit::planning_interface::MoveGroupInterface &move_group,
                const moveit::planning_interface::MoveGroupInterface::Plan &plan);

  /// destructor
  ~MoveitVisuals();

  /// A function to visualize planned move in RViz
  void showPlannedPath();

  /// add obstacle to planning scene. Attached/detached will happen later
  void addObstacle(const jaco_manipulation::PlanAndMoveArmGoalConstPtr &goal);

  /// attaches obstacle to robot. Trajectory is then planned with obstacle in mind
  void attachObstacle(const jaco_manipulation::PlanAndMoveArmGoalConstPtr &goal);

  /// detaches obstacle from robot
  void detachObstacle(const jaco_manipulation::PlanAndMoveArmGoalConstPtr &goal);

  /// remove obstacle from planning scene by its id
  void removeObstacle(const std::string id);

  /// Wipe all obstacles that came from the kinect (or ohter vision system), basically everything except table
  void wipeKinectObstacles();

  /// get number of obstacles currently in the scene
  unsigned long numOfObstacles();

  /**
   * Get obstacles in planning scene. in ROOT frame (moveits planning frame)
   * @return
   */
  const std::vector<jaco_manipulation::BoundingBox>& getObstacles();

 private:

  /// nodehandle
  ros::NodeHandle &nh_;

  /// transform listener
  tf::TransformListener tf_listener_;

  /// publishes changes to planning scene
  ros::Publisher planning_scene_diff_publisher_;

  /// moveit planning scene
  moveit_msgs::PlanningScene planning_scene_;

  /**
   * All obstacles in planning scene
   */
  std::vector<jaco_manipulation::BoundingBox> obstacles_;

  /**
   * MoveIt visual tools
  */
  moveit_visual_tools::MoveItVisualTools visual_tools_;

  /**
   * The move_group variable.
  */
  moveit::planning_interface::MoveGroupInterface &move_group_;

  /**
   * The planning scene interface.
   * This we use to add obstacles. These obstacles are the planes.
  */
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  /**
   * The plan variable.
  */
  const moveit::planning_interface::MoveGroupInterface::Plan &plan_;

  /**
   * the next object to be attached.
   */
  std::map<std::string, moveit_msgs::AttachedCollisionObject> to_be_attached;

  /**
   * A function to prepare MoveIt! Visual Tools in RViz
  */
  void prepMoveItVisualTools();

  /**
   * Shows all objects: attached and planning scene obstacles
   */
  void showStatus();

  /**
   * Adds table surface to planning scene on startup
   */
  void addTableObstacle();
};
} // namespace visuals
} // namespace jaco_manipulation

#endif //PROJECT_MOVEIT_VISUALS_H
