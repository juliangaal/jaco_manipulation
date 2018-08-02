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
#ifndef GRASP_POSE_GENERATOR_H_
#define GRASP_POSE_GENERATOR_H_

#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <std_srvs/Empty.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <jaco_manipulation/GenerateGraspPoses.h>

namespace jaco_manipulation {
/**
 * Provides a ROS service to generate a graps pose from a point in 3D.
 * Assumes that the point is within bounds and reachable.
 */
class GraspPoseGenerator {

 public:
  /**
   * Constructor.
   */
  GraspPoseGenerator();

  /**
   * Destructor.
   */
  ~GraspPoseGenerator();

  /**
   * A listener for the transform information.
   */
  tf::TransformListener *tf_listener_;

 protected:

  /**
   * ROS server that returns a grasp pose when object's location is given.
   */
  ros::ServiceServer server_;

  /**
   * Server callback.
   */
  bool serverCB(jaco_manipulation::GenerateGraspPosesRequest &request,
                jaco_manipulation::GenerateGraspPosesResponse &response);

  /**
   * Convenience function.
   */
  inline float dist3D(const geometry_msgs::Point p1, const geometry_msgs::Point p2);

  /**
   * Convenience function to transform a point to the base_link frame.
   */
  geometry_msgs::PointStamped transformPointToBaseLink(const geometry_msgs::PointStamped &in_pt);

  /**
   * A common nodehandle.
   */
  ros::NodeHandle n_;

  /**
   * The generated poses are stored in these variables.
   */
  geometry_msgs::PoseStamped target_pose_top_;
  geometry_msgs::PoseStamped target_pose_1_;
  geometry_msgs::PoseStamped target_pose_2_;
  geometry_msgs::PoseStamped target_pose_3_;
};
}
#endif /* JACO_HAND_POSE_GENERATOR_H_ */
