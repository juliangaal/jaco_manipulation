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

#include <jaco_manipulation/grasps/grasp_pose_generator.h>

using namespace jaco_manipulation::grasps;

GraspPoseGenerator::GraspPoseGenerator()
  : tf_listener_(nh_, ros::Duration(10)),
    absolute_height_top_grasp_(min_height_top_grasp_),
    absolute_height_front_grasp_(min_height_front_grasp_) {}

void GraspPoseGenerator::adjustPose(geometry_msgs::PoseStamped &pose,
                                    const BoundingBox &box,
                                    const GraspType type) {
  setAbsoluteHeight(box);

  switch(type) {
    case TOP_GRASP:
      adjustPosition(pose, box, TOP_GRASP);
      adjustToTopOrientation(pose);
      adjustOrientationToShape(pose, box);
      break;
    case TOP_DROP:
      adjustPosition(pose, box, TOP_DROP);
      adjustToTopOrientation(pose);
      break;
    case FRONT_GRASP:
      adjustPosition(pose, box, FRONT_GRASP);
      transformGoalIntoRobotFrame(pose, box);
      adjustToFrontOrientation(pose); // TODO which height, when bounding box centroid above min_height_front_grasp_?!
      break;
    default:
      adjustPosition(pose, box, TOP_DROP);
      adjustToTopOrientation(pose);
  }
}

void GraspPoseGenerator::adjustPose(geometry_msgs::PoseStamped &pose, const GraspType type) {
  switch(type) {
    case TOP_GRASP:
      transformGoalIntoRobotFrame(pose);
      adjustToTopOrientation(pose);
      break;
    case TOP_DROP:
      transformGoalIntoRobotFrame(pose);
      adjustToTopOrientation(pose);
      break;
    case FRONT_GRASP:
      transformGoalIntoRobotFrame(pose);
      adjustToFrontOrientation(pose); // TODO set height, move gripper to accomodate front grasp
      break;
    default:
      transformGoalIntoRobotFrame(pose);
      adjustToTopOrientation(pose);
  }
}

void GraspPoseGenerator::adjustPosition(geometry_msgs::PoseStamped &pose,
                                        const BoundingBox &box,
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

  transformGoalIntoRobotFrame(pose, box);

  switch(type) {
    case TOP_GRASP:
      adjustHeightForTopPose(pose, box);
      break;
    case TOP_DROP:
      adjustHeightForTopDropPose(pose, box);
      break;
    case FRONT_GRASP:
      pose.pose.position.z = box.point.z;
      adjustPositionForFrontPose(pose);
      adjustHeightForFrontPose(pose, box);
      break;
    default:
      adjustToTopOrientation(pose);
  }
}

double GraspPoseGenerator::degToRad(const double &amount) {
  return (amount * M_PI)/180.;
}

void GraspPoseGenerator::rotatePose(geometry_msgs::PoseStamped &pose, double amount, Rotation r_type) {
  tf::Quaternion q_orig, q_rot, q_new;

  const double rotation = degToRad(amount);
  switch(r_type) {
    case Rotation::ROLL:
      q_rot = roll(rotation);
      break;
    case Rotation::PITCH:
      q_rot = pitch(rotation);
      break;
    case Rotation::YAW:
      q_rot = yaw(rotation);
  }

  quaternionMsgToTF(pose.pose.orientation , q_orig);
  q_new = q_rot * q_orig;
  q_new.normalize();
  quaternionTFToMsg(q_new, pose.pose.orientation);
}

tf::Quaternion GraspPoseGenerator::yaw(double amount) {
  return tf::createQuaternionFromRPY(0, 0, amount);
}

tf::Quaternion GraspPoseGenerator::roll(double amount) {
  return tf::createQuaternionFromRPY(0, amount, 0);
}

tf::Quaternion GraspPoseGenerator::pitch(double amount) {
  return tf::createQuaternionFromRPY(amount, 0, 0);
}

void GraspPoseGenerator::transformGoalIntoRobotFrame(geometry_msgs::PoseStamped &pose,
                                                     const BoundingBox &box) {
  geometry_msgs::PointStamped out_pt;
  geometry_msgs::PointStamped in_pt;
  in_pt.header = box.header;
  in_pt.point = pose.pose.position;

  // transform point into root frame
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


void GraspPoseGenerator::transformGoalIntoRobotFrame(geometry_msgs::PoseStamped &pose) {
  geometry_msgs::PointStamped out_pt;
  geometry_msgs::PointStamped in_pt;
  in_pt.header = pose.header;
  in_pt.point = pose.pose.position;

  // transform point
  try {
    tf_listener_.waitForTransform("root", pose.header.frame_id, pose.header.stamp, ros::Duration(1));
    tf_listener_.transformPoint("root", in_pt, out_pt);
  }
  catch (tf::TransformException &exception) {
    ROS_INFO_STREAM("Transform failed. Why? - " << exception.what());
  }
  pose.pose.position = out_pt.point;

  ROS_INFO("Transfrm: \"%s\" (%f,%f,%f) -> \"%s\" (%f,%f,%f)",
           pose.header.frame_id.c_str(),
           in_pt.point.x,
           in_pt.point.y,
           in_pt.point.z,
           "root",
           pose.pose.position.x,
           pose.pose.position.y,
           pose.pose.position.z);
}

void GraspPoseGenerator::setAbsoluteHeight(const BoundingBox &box) {
  geometry_msgs::PointStamped out_pt;
  geometry_msgs::PointStamped in_pt;
  in_pt.header = box.header;

  geometry_msgs::PoseStamped pose; // empty pose in base link
  in_pt.point = pose.pose.position;

  // transform point into root frame
  try {
    tf_listener_.waitForTransform("root", box.header.frame_id, box.header.stamp, ros::Duration(1));
    tf_listener_.transformPoint("root", in_pt, out_pt);
  }
  catch (tf::TransformException &exception) {
    ROS_INFO_STREAM("Absolute height transform failed. Why? - " << exception.what());
  }

  const auto height_diff = std::fabs(out_pt.point.z - in_pt.point.z);
  // subtract height diff and add 2.5cm, so it's always above!
  absolute_height_top_grasp_ -= height_diff;
  absolute_height_top_grasp_ += 2.5_cm;

  absolute_height_front_grasp_ -= height_diff/2;
}

void GraspPoseGenerator::adjustHeightForTopPose(geometry_msgs::PoseStamped &pose,
                                                const BoundingBox &box) {
  if (pose.pose.position.z < absolute_height_top_grasp_) {
    ROS_WARN_STREAM("Correction: Top Grasp Pose too low: correcting to " << absolute_height_top_grasp_);
    pose.pose.position.z = absolute_height_top_grasp_;
  }
  // we adjust height according to width of bounding box
  // Because of inherent properties and the "angle" of the fingers, we have to adjust the height of the grasp pose
  // according to the size of the object.
  // function described by y = mx + b
  // m = delta y/ delta x = 4.5/6.5 = 0.85
  // b = 2.7
  // function (defined after x(width) > 0.065: y = mx + b
  constexpr double min = min_height_top_grasp_; // minimum height: current min
  constexpr double max = 25.5_cm; // maximum heihgt for gripping an object of around 13 cm x/y
  constexpr double delta_y = max - min;
  constexpr double delta_x = 13._cm - 7.2_cm;
  constexpr double m = delta_y / delta_x;
  constexpr double b = 20._cm - m * 7.2_cm;

  if ((box.dimensions.x > 7.2_cm && box.dimensions.x <= box.dimensions.y) ||
      (box.dimensions.y > 7.2_cm && box.dimensions.y <= box.dimensions.x)) {
    // min because we turn rotation for best fit in hand.
    // see adjustOrientationToShape
    const auto &y = pose.pose.position.z;
    auto x = std::min(box.dimensions.x, box.dimensions.y);
    auto height_correction = m*x + b;
    auto height_diff = std::fabs(height_correction - y);
    ROS_INFO("H/W Fix : %f -> %f for width %f and diff %f", y, y + height_diff, x, height_diff);
    ROS_INFO("mx + b %f %f %f", m, x, b);
    pose.pose.position.z += height_diff;
  }
}

void GraspPoseGenerator::adjustHeightForTopDropPose(geometry_msgs::PoseStamped &pose, const BoundingBox &box) {
  adjustHeightForTopPose(pose, box);
  pose.pose.position.z += drop_offset_;
}

void GraspPoseGenerator::adjustHeightForFrontPose(geometry_msgs::PoseStamped &pose, const BoundingBox &box) {
  if (pose.pose.position.z < absolute_height_front_grasp_) {
    ROS_WARN_STREAM("Correction: Front Grasp Pose too low: correcting to" << min_height_front_grasp_);
    pose.pose.position.z = absolute_height_front_grasp_;
  }
}

void GraspPoseGenerator::adjustPositionForFrontPose(geometry_msgs::PoseStamped &pose) {
  // we have to move pose to accomodate for the offset from jaco_link_hand to finger tips
  pose.pose.position.x += (pose.pose.position.x >= 0) ? -grasp_offset_ : grasp_offset_;
}

void GraspPoseGenerator::adjustOrientationToShape(geometry_msgs::PoseStamped &pose,
                              const BoundingBox &box) {
// Before we can adjust the height of the pose, we need to know about the ortientation of the objects
// In our case we only care about two orientations, because the anchoring system
// can't publish a bounding box with an accurate orientation. So, our scenario is reduced to these two orientations
//     ______          ____
//  y |_____|  and  y |   |
//       x            |___|
//                      x
// We need to rotate the gripper 90deg if the length in x is longer than the length in y
  if (box.dimensions.x > box.dimensions.y)
    rotatePose(pose, 90._deg, Rotation::YAW);
}

void GraspPoseGenerator::adjustToTopOrientation(geometry_msgs::PoseStamped &pose) {
  // Direction vector of z-axis. Should match root z-axis orientation
  tf::Vector3 z_axis(
      0,
      0,
      1
  );

  /**
   * Direction vector of our new x-axis, NOT defined in relation to y and z, but always parallel to the y axis in reality.
  */
  tf::Vector3 x_axis(
      1,
      0,
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
  adjustToTopOrientation(pose);

  // roll the pose to achieve front grasp
  double amount = (pose.pose.position.x < 0) ? 90.0_deg : -90._deg;
  rotatePose(pose, -amount, Rotation::YAW);
  rotatePose(pose, amount, Rotation::ROLL);
//
//  // adjust height, if smaller than minimal allowance
//  if (pose.pose.position.z < min_height_front_grasp_)
//    pose.pose.position.z = min_height_front_grasp_;

  ROS_INFO("FrontFix: Pose now (%f,%f,%f) ; (%f,%f,%f,%f)",
           pose.pose.position.x,
           pose.pose.position.y,
           pose.pose.position.z,
           pose.pose.orientation.x,
           pose.pose.orientation.y,
           pose.pose.orientation.z,
           pose.pose.orientation.w);
}
