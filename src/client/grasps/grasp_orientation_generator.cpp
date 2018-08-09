//
// Created by chitt on 8/9/18.
//

#include <jaco_manipulation/client/grasps/grasp_orientation_generator.h>
#include <tf/tf.h>

using namespace jaco_manipulation::client::grasps;

void GraspOrientationGenerator::adjustOrientation(const TopGraspOrientation &grasp, geometry_msgs::PoseStamped &pose) {
  // Direction vector of z-axis. Should match root z-axis orientation
  tf::Vector3 z_axis(
      0,
      0,
      grasp.rotationZ_
  );

  /**
   * Direction vector of our new x-axis, defined in relation to y and z. Dynamically calculated with current Pose.
   * z is ignored, because we want the grasp pose to always be horizontal
  */
  tf::Vector3 x_axis(
      pose.pose.position.x * grasp.rotationX_,
      pose.pose.position.y * grasp.rotationY_,
      0
  );
  x_axis.normalize();

  // Calculate missing y-axis from defined z and x axis
  tf::Vector3 y_axis;
  y_axis = z_axis.cross(x_axis);

  tf::Matrix3x3 top_grasp_orientation(
      x_axis.x(), x_axis.y(), x_axis.z(),
      y_axis.x(), y_axis.y(), y_axis.z(),
      z_axis.x(), z_axis.y(), z_axis.z()
  );

  // convert orientation matrix to quaternion
  tf::Quaternion top_grasp_quaternion;
  top_grasp_orientation.getRotation(top_grasp_quaternion);
  tf::quaternionTFToMsg(top_grasp_quaternion, pose.pose.orientation);

  ROS_INFO("Goal Fix: Pose now (%f,%f,%f) ; (%f,%f,%f,%f)",
           pose.pose.position.x,
           pose.pose.position.y,
           pose.pose.position.z,
           pose.pose.orientation.x,
           pose.pose.orientation.y,
           pose.pose.orientation.z,
           pose.pose.orientation.w);
}

void GraspOrientationGenerator::adjustOrientation(const FrontGraspOrientation &grasp,
                                                  geometry_msgs::PoseStamped &pose) {
  // Direction vector of z-axis. WARN: The z-axis will become the new x-axis for front grip
  tf::Vector3 z_axis(
      0,
      0,
      grasp.rotationZ_
  );

  /**
   * Direction vector of our new x-axis, defined in relation to y and z. Dynamically calculated with current Pose.
   * z is ignored, because we want the grasp pose to always be horizontal
  */
  tf::Vector3 x_axis(
      pose.pose.position.x * grasp.rotationX_,
      pose.pose.position.y * grasp.rotationY_,
      0
  );
  x_axis.normalize();

  // Calculate missing y-axis from defined z and x axis
  tf::Vector3 y_axis;
  y_axis = x_axis.cross(y_axis);

  tf::Matrix3x3 top_grasp_orientation(
      z_axis.x(), z_axis.y(), z_axis.z(),
      y_axis.x(), y_axis.y(), y_axis.z(),
      x_axis.x(), x_axis.y(), x_axis.z()
  );

  // convert orientation matrix to quaternion
  tf::Quaternion top_grasp_quaternion;
  top_grasp_orientation.getRotation(top_grasp_quaternion);
  tf::quaternionTFToMsg(top_grasp_quaternion, pose.pose.orientation);

  ROS_INFO("Goal Fix: Pose now (%f,%f,%f) ; (%f,%f,%f,%f)",
           pose.pose.position.x,
           pose.pose.position.y,
           pose.pose.position.z,
           pose.pose.orientation.x,
           pose.pose.orientation.y,
           pose.pose.orientation.z,
           pose.pose.orientation.w);
}
