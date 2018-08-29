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

#ifndef PROJECT_GRASP_ORIENTATION_GENERATOR_H
#define PROJECT_GRASP_ORIENTATION_GENERATOR_H

#include <geometry_msgs/PoseStamped.h>
#include <jaco_manipulation/goals/goal_input.h>
#include <jaco_manipulation/BoundingBox.h>
#include <tf/transform_listener.h>
#include <ros/node_handle.h>

namespace jaco_manipulation {
namespace grasps {

/// enum to define all grasp types
enum GraspType { TOP_GRASP, TOP_DROP, FRONT_GRASP, LEFT_GRASP, RIGHT_GRASP };

/**
 * GraspPoseGenerator generates poses according to specified pose in GraspType enum
 */
class GraspPoseGenerator {
 public:
  /**
   * Constructor
   */
  GraspPoseGenerator();

  /**
   * default destructor
   */
  ~GraspPoseGenerator() = default;

  /**
   * Adjusts Pose
   * @param pose input pose
   * @param box bounding box to be adjusted to
   * @param type grasp type yo adjust to
   */
  void adjustPose(geometry_msgs::PoseStamped &pose,
                  const jaco_manipulation::BoundingBox &box,
                  const GraspType type);

  /**
 * Adjusts Pose
 * @param pose input pose
 * @param type grasp type yo adjust to
 */
  void adjustPose(geometry_msgs::PoseStamped &pose,
                  const GraspType type);

 private:
  /**
   * Adjusts height for top pose
   * @param pose input pose
   * @param box input bounding box
   */
  void adjustHeightForTopPose(geometry_msgs::PoseStamped &pose,
                              const jaco_manipulation::BoundingBox &box);

  /**
   * Adjusts height for top pose
   * @param pose input pose
   * @param box input bounding box
   */
  void adjustHeightForTopDropPose(geometry_msgs::PoseStamped &pose,
                                  const jaco_manipulation::BoundingBox &box);

  /**
   * Adjusts height for front pose
   * @param pose input pose
   * @param box input bounding box
   */
  void adjustHeightForFrontPose(geometry_msgs::PoseStamped &pose,
                                const jaco_manipulation::BoundingBox &box);

  /**
   * Adjust position from bounding box centroid
   * @param pose input pose
   * @param box input bounding box
   * @param type grasp type
   */
  void adjustPosition(geometry_msgs::PoseStamped &pose,
                      const jaco_manipulation::BoundingBox &box,
                      const GraspType type);

  /**
   * Adjusts orientation for top pose
   * @param pose input pose
   */
  void adjustToTopOrientation(geometry_msgs::PoseStamped &pose);

  /**
  * Adjusts orientation for front pose
  * @param pose input pose
  */
  void adjustToFrontOrientation(geometry_msgs::PoseStamped &pose);

  /**
  * Adjusts position for front pose: offset for gripping
  * @param pose input pose
  */
  void adjustPositionForFrontPose(geometry_msgs::PoseStamped &pose);

  /**
   * transforms goal into robot frame
   * @param pose input pose
   * @param box input bounding box
   */
  void transformGoalIntoRobotFrame(geometry_msgs::PoseStamped &pose,
                                   const jaco_manipulation::BoundingBox &box);

  /**
 * transforms goal into robot frame
 * @param pose input pose
 */
  void transformGoalIntoRobotFrame(geometry_msgs::PoseStamped &pose);

  /// Node Handle
  ros::NodeHandle n_;

  /// transform listener
  tf::TransformListener tf_listener_;

  /// Minimum height for top grasp
  constexpr static double min_height_top_grasp = 0.175026;

  /// Minimum height for front grasp
  constexpr static double min_height_front_grasp = 0.011;

  /// offset that is added for dropping
  constexpr static double drop_offset_ = 0.03;

  /// offset to stack two objects
  constexpr static double stack_offset_ = 0.01;

  /// offset for grasping: distance between jaco_link_hand and palm
  constexpr static double grasp_offset_ = 0.145;
};

} // namespace grasps
} // namespace jaco_manipulation

#endif //PROJECT_GRASP_ORIENTATION_GENERATOR_H
