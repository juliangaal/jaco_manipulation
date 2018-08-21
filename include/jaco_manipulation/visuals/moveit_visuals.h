//
// Created by chitt on 8/13/18.
//

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

class MoveitVisuals {
 public:
  MoveitVisuals() = delete;
  MoveitVisuals(ros::NodeHandle &nh, const std::string frame,
                moveit::planning_interface::MoveGroupInterface &move_group,
                const moveit::planning_interface::MoveGroupInterface::Plan &plan);
  ~MoveitVisuals();

  /**
   * A function to visualize planned move in RViz
   */
  void showPlannedPath();

  void addObstacle(const jaco_manipulation::PlanAndMoveArmGoalConstPtr &goal);

  void attachObstacle(const jaco_manipulation::BoundingBox &box);
  void detachObstacle(const jaco_manipulation::BoundingBox &box);
  void removeObstacle(const std::string id);

 private:

  ros::NodeHandle &nh_;

  tf::TransformListener tf_listener_;

  ros::Publisher planning_scene_diff_publisher_;

  moveit_msgs::PlanningScene planning_scene_;

  /**
   * MoveIt visual tools
  */
  moveit_visual_tools::MoveItVisualTools visual_tools_;

  /**
   * The move_group variable.
  */
  moveit::planning_interface::MoveGroupInterface &move_group_;

//  moveit_msgs::PlanningScene planning_scene_;

  /**
   * The planning scene interface.
   * This we use to add obstacles. These obstacles are the planes.
  */
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  /**
   * The plan variable.
  */
  const moveit::planning_interface::MoveGroupInterface::Plan &plan_;

  std::map<std::string, moveit_msgs::AttachedCollisionObject> to_be_attached;
  /**
   * A function to prepare MoveIt! Visual Tools in RViz
  */
  void prepMoveItVisualTools();

  void addTableObstacle();
};
} // namespace visuals
} // namespace jaco_manipulation

#endif //PROJECT_MOVEIT_VISUALS_H
