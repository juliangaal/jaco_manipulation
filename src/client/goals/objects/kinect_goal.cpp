//
// Created by julian on 8/6/18.
//

#include <jaco_manipulation/client/goals/objects/kinect_goal.h>
#include <ros/console.h>
#include <tf/tf.h>

using namespace jaco_manipulation::client::goals::objects;

KinectGoal::KinectGoal(const kinect_goal_definitions::LimitedPose &grasp_pose_goal,
                       jaco_manipulation::client::grasps::GraspType grasp,
                       const std::string &description) {
  description_ = description;

  goal_.pose_goal.header.frame_id = planning_frame_;
  goal_.goal_type = "goal";

  goal_.pose_goal.pose.position.x = grasp_pose_goal.x;
  goal_.pose_goal.pose.position.y = grasp_pose_goal.y;
  goal_.pose_goal.pose.position.z = grasp_pose_goal.z;

  adjustOrientation(grasp);
}

KinectGoal::KinectGoal(const kinect_goal_definitions::BoundingBox &bounding_box_goal,
                       jaco_manipulation::client::grasps::GraspType grasp,
                       const std::string &description) {
  description_ = description + " \"" + bounding_box_goal.description + "\"";

  goal_.pose_goal.header.frame_id = planning_frame_;
  goal_.goal_type = "goal";

  adjustPoseToCenterOfObject(bounding_box_goal);
  adjustOrientation(grasp);
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

  goal_.pose_goal.pose.position.x += (bounding_box.x >= 0.0) ? width_adj : -width_adj;
  goal_.pose_goal.pose.position.y += (bounding_box.y >= 0.0) ? -length_adj : length_adj;

  ROS_INFO("Box Fix : (%f %f %f) -> (%f %f %f)",
                                      bounding_box.x,
                                      bounding_box.y,
                                      bounding_box.z,
                                      goal_.pose_goal.pose.position.x,
                                      goal_.pose_goal.pose.position.y,
                                      goal_.pose_goal.pose.position.z);
}

void KinectGoal::adjustOrientation(jaco_manipulation::client::grasps::GraspType grasp) {
  grasp_orientation_generator_.adjustOrientation(goal_.pose_goal, grasp);
}
