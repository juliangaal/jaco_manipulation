//
// Created by chitt on 8/9/18.
//

#include <jaco_manipulation/grasps/grasp_pose_generator.h>
#include <tf/tf.h>

using namespace jaco_manipulation::grasps;

void GraspPoseGenerator::adjustPose(geometry_msgs::PoseStamped &pose,
                                    const jaco_manipulation::goals::goal_input::BoundingBox &box,
                                    const GraspType type) {
  switch(type) {
    case TOP_GRASP:
      adjustPosition(pose, box, TOP_GRASP);
      adjustToTopOrientation(pose);
      break;
    case TOP_DROP:
      adjustPosition(pose, box, TOP_DROP);
      adjustToTopOrientation(pose);
      break;
    case FRONT_GRASP:
      adjustToFrontOrientation(pose);
      break;
    default:
      adjustToTopOrientation(pose);
  }
}

void GraspPoseGenerator::adjustPosition(geometry_msgs::PoseStamped &pose,
                                        const jaco_manipulation::goals::goal_input::BoundingBox &box,
                                        const GraspType type) {
  pose.pose.position.x = box.x;
  pose.pose.position.y = box.y;
  pose.pose.position.z = box.z;

  const double width_adj = box.width * 0.5;
  const double length_adj = box.length * 0.5;

  pose.pose.position.x += (box.x >= 0.0) ? width_adj : -width_adj;
  pose.pose.position.y += (box.y >= 0.0) ? -length_adj : length_adj;

  ROS_INFO("Box Fix : (%f %f %f) -> (%f %f %f)",
           box.x,
           box.y,
           box.z,
           pose.pose.position.x,
           pose.pose.position.y,
           pose.pose.position.z);

  switch(type) {
    case TOP_GRASP:
      adjustHeightForTopPose(pose, box);
      break;
    case TOP_DROP:
      adjustHeightForTopDropPose(pose, box);
      break;
    default:
      adjustToTopOrientation(pose);
  }

}

void GraspPoseGenerator::adjustHeightForTopPose(geometry_msgs::PoseStamped &pose,
                                                const jaco_manipulation::goals::goal_input::BoundingBox &box) {
  // adjust height, if smaller than minimal allowance
  if (pose.pose.position.z < min_height_top_grasp)
    pose.pose.position.z = min_height_top_grasp;

  double height_adj = 0.0;

  if (box.height > pose.pose.position.z)
    height_adj = std::fabs(box.height - pose.pose.position.z);

  pose.pose.position.z += height_adj;
}

void GraspPoseGenerator::adjustHeightForTopDropPose(geometry_msgs::PoseStamped &pose,
                                                    const jaco_manipulation::goals::goal_input::BoundingBox &box) {
  adjustHeightForTopPose(pose, box);
  pose.pose.position.z += drop_offset_;
}

void GraspPoseGenerator::adjustToTopOrientation(geometry_msgs::PoseStamped &pose) {
  // Direction vector of z-axis. Should match root z-axis orientation
  tf::Vector3 z_axis(
      0,
      0,
      1
  );

  /**
   * Direction vector of our new x-axis, defined in relation to y and z. Dynamically calculated with current Pose.
   * z is ignored, because we want the grasp pose to always be horizontal
  */
  tf::Vector3 x_axis(
      pose.pose.position.x, //* grasp.rotationX_,
      pose.pose.position.y,// * grasp.rotationY_,
      0
  );
  x_axis.normalize();

  // Calculate missing y-axis from defined z and x axis
  tf::Vector3 y_axis;
  y_axis = z_axis.cross(x_axis);

  tf::Matrix3x3 top_grasp_orientation(
      x_axis.x(), y_axis.x(), z_axis.x(),
      x_axis.y(), y_axis.y(), z_axis.y(),
      x_axis.z(), y_axis.z(), z_axis.z()
  );

  // convert orientation matrix to quaternion
  tf::Quaternion top_grasp_quaternion;
  top_grasp_orientation.getRotation(top_grasp_quaternion);
  tf::quaternionTFToMsg(top_grasp_quaternion, pose.pose.orientation);

  ROS_INFO("Top Fix : Pose now (%f,%f,%f) ; (%f,%f,%f,%f)",
           pose.pose.position.x,
           pose.pose.position.y,
           pose.pose.position.z,
           pose.pose.orientation.x,
           pose.pose.orientation.y,
           pose.pose.orientation.z,
           pose.pose.orientation.w);
}

void GraspPoseGenerator::adjustToFrontOrientation(geometry_msgs::PoseStamped &pose) {
  // Direction vector of z-axis. WARN: The z-axis will become the new x-axis for front grip
  tf::Vector3 z_axis(
      pose.pose.position.x,
      pose.pose.position.y,
      0
  );
  z_axis *= -1; // z-axis direction for front grasp should be opposite of x-axis direction of top grasp
  z_axis.normalize();

  /**
   * Direction vector of our new x-axis, defined in relation to y and z. Dynamically calculated with current Pose.
   * z is ignored, because we want the grasp pose to always be horizontal
  */
  tf::Vector3 y_axis(
      0,
      0,
      1
  );

  // Calculate missing y-axis from defined z and x axis
  tf::Vector3 x_axis;
  x_axis = y_axis.cross(z_axis);

  tf::Matrix3x3 front_grasp_orientation(
      x_axis.x(), y_axis.x(), z_axis.x(),
      x_axis.y(), y_axis.y(), z_axis.y(),
      x_axis.z(), y_axis.z(), z_axis.z()
  );

  // convert orientation matrix to quaternion
  tf::Quaternion front_grasp_quaternion;
  front_grasp_orientation.getRotation(front_grasp_quaternion);
  tf::quaternionTFToMsg(front_grasp_quaternion, pose.pose.orientation);

  // adjust height, if smaller than minimal allowance
  if (pose.pose.position.z < min_height_front_grasp)
    pose.pose.position.z = min_height_front_grasp;

  ROS_INFO("Front Fix: Pose now (%f,%f,%f) ; (%f,%f,%f,%f)",
           pose.pose.position.x,
           pose.pose.position.y,
           pose.pose.position.z,
           pose.pose.orientation.x,
           pose.pose.orientation.y,
           pose.pose.orientation.z,
           pose.pose.orientation.w);
}
