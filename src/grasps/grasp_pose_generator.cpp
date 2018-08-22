//
// Created by chitt on 8/9/18.
//

#include <jaco_manipulation/grasps/grasp_pose_generator.h>
#include <tf/tf.h>

using namespace jaco_manipulation::grasps;

GraspPoseGenerator::GraspPoseGenerator() : tf_listener_(n_, ros::Duration(10)) {}

void GraspPoseGenerator::adjustPose(geometry_msgs::PoseStamped &pose,
                                    const jaco_manipulation::BoundingBox &box,
                                    const GraspType type) {
  switch(type) {
    case TOP_GRASP:
      adjustPosition(pose, box, TOP_GRASP);
      transformGoalIntoRobotFrame(pose, box);
      adjustToTopOrientation(pose);
      break;
    case TOP_DROP:
      adjustPosition(pose, box, TOP_DROP);
      transformGoalIntoRobotFrame(pose, box);
      adjustToTopOrientation(pose);
      break;
    case FRONT_GRASP:
      adjustPosition(pose, box, FRONT_GRASP);
      transformGoalIntoRobotFrame(pose, box);
      adjustToFrontOrientation(pose); // TODO set height, move gripper to accomodate front grasp
      break;
    default:
      adjustPosition(pose, box, TOP_DROP);
      transformGoalIntoRobotFrame(pose, box);
      adjustToTopOrientation(pose);
  }
}

void GraspPoseGenerator::transformGoalIntoRobotFrame(geometry_msgs::PoseStamped &pose,
                                                     const jaco_manipulation::BoundingBox &box) {
  geometry_msgs::PointStamped out_pt;
  geometry_msgs::PointStamped in_pt;
  in_pt.header = box.header;
  in_pt.point = pose.pose.position;

  // transform point
  try {
    tf_listener_.waitForTransform("root", box.header.frame_id, box.header.stamp, ros::Duration(1));
    tf_listener_.transformPoint("root", in_pt, out_pt);
  }
  catch (tf::TransformException &exception) {
    ROS_INFO_STREAM("Transform failed. Why? - " << exception.what());
  }
  pose.pose.position = out_pt.point;

  ROS_INFO("Transfrm: \"%s\" (%f,%f,%f) -> \"%s\" (%f,%f,%f)",
           box.header.frame_id.c_str(),
           in_pt.point.x,
           in_pt.point.y,
           in_pt.point.z,
           "root",
           pose.pose.position.x,
           pose.pose.position.y,
           pose.pose.position.z);
}

void GraspPoseGenerator::adjustPosition(geometry_msgs::PoseStamped &pose,
                                        const jaco_manipulation::BoundingBox &box,
                                        const GraspType type) {
  pose.pose.position.x = box.point.x;
  pose.pose.position.y = box.point.y;
  pose.pose.position.z = box.point.z + box.dimensions.z*0.5 + grasp_offset_;

  ROS_INFO("Box Fix : (%f %f %f) -> (%f %f %f)",
           box.point.x,
           box.point.y,
           box.point.z,
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
    case FRONT_GRASP:
      adjustHeightForFrontPose(pose, box);
      break;
    default:
      adjustToTopOrientation(pose);
  }

}

void GraspPoseGenerator::adjustHeightForTopPose(geometry_msgs::PoseStamped &pose,
                                                const jaco_manipulation::BoundingBox &box) {
  // adjust height, if smaller than minimal allowance
  if (pose.pose.position.z < min_height_top_grasp)
    pose.pose.position.z = min_height_top_grasp;
}

void GraspPoseGenerator::adjustHeightForTopDropPose(geometry_msgs::PoseStamped &pose,
                                                    const jaco_manipulation::BoundingBox &box) {
  adjustHeightForTopPose(pose, box);
  pose.pose.position.z += drop_offset_;
}

void GraspPoseGenerator::adjustHeightForFrontPose(geometry_msgs::PoseStamped &pose,
                                                    const jaco_manipulation::BoundingBox &box) {
  adjustHeightForTopPose(pose, box);
  if (box.point.z < min_height_front_grasp)
    pose.pose.position.z = min_height_front_grasp;
  else
    pose.pose.position.z = box.point.z - std::fabs(min_height_front_grasp - box.point.z);
}

void GraspPoseGenerator::adjustPoseForFrontPose(geometry_msgs::PoseStamped &pose) {
  tf::Vector3 change_vector(
    pose.pose.position.x,
    pose.pose.position.y,
    0
  );
  change_vector.normalize();

  // grasp offset
  tf::Vector3 move_vector(
    0.145,
    0.145,
    0
  );

  // adjust change vector according to direction vector
  move_vector.setX(move_vector.x() * change_vector.x());
  move_vector.setY(move_vector.y() * change_vector.y());

  tf::Vector3 curr_vector(
    pose.pose.position.x,
    pose.pose.position.y,
    pose.pose.position.z
  );

  tf::Vector3 adj_vec = curr_vector - move_vector;

  ROS_INFO("Front Move: Pose (%f,%f,%f) -> (%f,%f,%f)",
           pose.pose.position.x,
           pose.pose.position.y,
           pose.pose.position.z,
           adj_vec.x(),
           adj_vec.y(),
           adj_vec.z());

  pose.pose.position.x = adj_vec.x();
  pose.pose.position.y = adj_vec.y();
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
  // adds offset in direction of grip
  adjustPoseForFrontPose(pose);

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
