//
// Created by julian on 8/6/18.
//

#include <jaco_manipulation/client/goals/objects/object_goal.h>
#include <ros/console.h>
#include <tf/tf.h>

using namespace jaco_manipulation::client::goals::objects;

KinectGoal::KinectGoal(const kinect_goal_definitions::LimitedPose &grasp_pose_goal, const std::string &description) {
  description_ = description;

  goal_.pose_goal.header.frame_id = planning_frame_;
  goal_.goal_type = "goal";

  goal_.pose_goal.pose.position.x = grasp_pose_goal.x;
  goal_.pose_goal.pose.position.y = grasp_pose_goal.y;
  goal_.pose_goal.pose.position.z = grasp_pose_goal.z;

  adjustPoseOrientationToAbsoluteOrientation();
  PoseGoal::adjustHeight();
}

KinectGoal::KinectGoal(const kinect_goal_definitions::BoundingBox &bounding_box_goal, const std::string &description) {
  description_ = description + " \"" + bounding_box_goal.description + "\"";

  goal_.pose_goal.header.frame_id = planning_frame_;
  goal_.goal_type = "goal";

  adjustPoseToCenterOfObject(bounding_box_goal);
  adjustPoseOrientationToAbsoluteOrientation();

  PoseGoal::adjustHeight();
}

jaco_manipulation::PlanAndMoveArmGoal KinectGoal::goal() const {
  return PoseGoal::goal();
}

void KinectGoal::adjustPoseToCenterOfObject(const kinect_goal_definitions::BoundingBox &bounding_box) {
  // TODO BOUNDING BOX CALCULATED BY TOP RIGHT CORNER IN COORDINATE SYSTEM. CHANGE AFTER FEEDBACK WITH ANDREAS
  goal_.pose_goal.pose.position.x = bounding_box.x;
  goal_.pose_goal.pose.position.y = bounding_box.y;
  goal_.pose_goal.pose.position.z = bounding_box.z;

  const double width_adj = bounding_box.width * 0.5;
  const double length_adj = bounding_box.length * 0.5;
  double height_adj = 0.0;
  // TODO this works even though it might seem to low. Jaco's, e.g. h=17cm is not real world height of 17cm, ASK CHITT
  // works even without dropping offset

  if (bounding_box.type == "drop_bounding_box") {
    if (bounding_box.height < min_height_) {
      height_adj = min_height_;
      ROS_WARN("Goal Fix: Drop Bounding Box too low. Adjusted.");
    } else {
      height_adj = min_height_ + std::fabs(bounding_box.height - min_height_);
    }

    height_adj += dropping_offset_;
    // TODO may be have to be adjusted once object is added to planning scene
  } else {
    if (bounding_box.height < min_height_) {
      height_adj = min_height_;
      ROS_INFO("Goal Fix: Drop Bounding Box too low. Adjusted.");
    } else {
      height_adj = min_height_ + std::fabs(bounding_box.height - min_height_);
    }
  }

  goal_.pose_goal.pose.position.x += (bounding_box.x >= 0.0) ? width_adj : -width_adj;
  goal_.pose_goal.pose.position.y += (bounding_box.y >= 0.0) ? -length_adj : length_adj;
  goal_.pose_goal.pose.position.z += height_adj;

  ROS_INFO("Box Fix : (%f %f %f) -> (%f %f %f)",
                                      bounding_box.x,
                                      bounding_box.y,
                                      bounding_box.z,
                                      goal_.pose_goal.pose.position.x,
                                      goal_.pose_goal.pose.position.y,
                                      goal_.pose_goal.pose.position.z);
}

void KinectGoal::adjustPoseOrientationToAbsoluteOrientation() {
  auto& pose = goal_.pose_goal;
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
      pose.pose.position.x,
      -pose.pose.position.y, // invert the y-position, because it inverts when the gripper moves towards the table top
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