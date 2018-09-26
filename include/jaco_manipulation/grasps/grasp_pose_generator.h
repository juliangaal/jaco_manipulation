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
#include <jaco_manipulation/units.h>
#include <tf/transform_listener.h>
#include <ros/node_handle.h>
#include <tf/tf.h>

namespace jaco_manipulation {
namespace grasps {

/// enum to define all grasp types
enum GraspType { TOP_GRASP, TOP_DROP, FRONT_GRASP, LEFT_GRASP, RIGHT_GRASP };

/// enum to define all Rotation types
enum Rotation { ROLL, PITCH, YAW };

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
   * Helper function to convert degree to radians
   * @param amount to be converted
   * @return double radians of degree amount
   */
  double degToRad(const double &amount);

  /**
   * Returns a yawed quaternion
   * @param amount to yaw by
   * @return rotated quaternion
   */
  tf::Quaternion yaw(double amount);

  /**
   * Returns a rolled quaternion
   * @param amount to roll by
   * @return rotated quaternion
   */
  tf::Quaternion roll(double amount);

  /**
   * Returns a pitched quaternion
   * @param amount to pitch by
   * @return rotated quaternion
   */
  tf::Quaternion pitch(double amount);

  /**
   * Rotates the pose
   * @param pose pose to rotate
   * @param amount amount in degree
   * @param r_type rotation type: Rotation::ROLL/PITCH/YAW
   */
  void rotatePose(geometry_msgs::PoseStamped &pose, double amount, Rotation r_type);

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
   * Adjusts the orientation of or grip according to shape of the bounding box
   * In our case we only care about two orientations, because the anchoring system
   * can't publish a bounding box with an accurate orientation. So, our scenario is reduced to these two orientations
   *  _____        ____
   * |____|  and  |   |
   *              |___|
   * We need to rotate the gripper 90deg if the length in x is longer than the length in y
   * @param pose to be rotated
   * @param box to take dimensions from
   */
  void adjustOrientationToShape(geometry_msgs::PoseStamped &pose,
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

  /**
   * Set absolute height: min_height_top_grasp + height to base_link by useing the bounding box
   * @param box point in root link
   */
  void setAbsoluteHeight(const BoundingBox &box);

  /// Node Handle
  ros::NodeHandle nh_;

  /// transform listener
  tf::TransformListener tf_listener_;

  /// Minimum height for top grasp in BASE_LINK
  constexpr static double min_height_top_grasp_ = 18._cm;

  /// min_height_jaco is defined in jaco's lonk (root). This will define it in base link
  double absolute_height_top_grasp_;

  /// Minimum height for front grasp
  constexpr static double min_height_front_grasp_ = .0_cm;

  /// min_height_front_grasp_ is defined in jaco's lonk (root). This will define it in base link
  double absolute_height_front_grasp_;

  /// offset that is added for dropping
  constexpr static double drop_offset_ = 4._cm;

  /// offset to stack two objects
  constexpr static double stack_offset_ = 1._cm;

  /// offset for grasping: distance between jaco_link_hand and palm
  constexpr static double grasp_offset_ = 17._cm;
};

} // namespace grasps
} // namespace jaco_manipulation

#endif //PROJECT_GRASP_ORIENTATION_GENERATOR_H
