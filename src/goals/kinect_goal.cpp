//
// Created by julian on 8/6/18.
//

#include <jaco_manipulation/goals/kinect_goal.h>
#include <ros/console.h>
#include <tf/tf.h>

using namespace jaco_manipulation::goals;

KinectGoal::KinectGoal(const goal_input::LimitedPose &grasp_pose_goal,
                       jaco_manipulation::grasps::GraspType grasp,
                       const std::string &description)
: requested_grasp_(grasp) {
  description_ = description;

  goal_.pose_goal.header.frame_id = planning_frame_;
  goal_.goal_type = "goal";

  goal_.pose_goal.pose.position.x = grasp_pose_goal.x;
  goal_.pose_goal.pose.position.y = grasp_pose_goal.y;
  goal_.pose_goal.pose.position.z = grasp_pose_goal.z;

//  adjustOrientation(grasp);
}

KinectGoal::KinectGoal(const goal_input::BoundingBox &bounding_box_goal,
                       jaco_manipulation::grasps::GraspType grasp,
                       const std::string &description)
: requested_grasp_(grasp) {
  description_ = description + " \"" + bounding_box_goal.description + "\"";

  goal_.pose_goal.header.frame_id = planning_frame_;
  goal_.goal_type = "goal";

//  adjustPoseToCenterOfObject(bounding_box_goal);
  adjustPose(grasp, bounding_box_goal);
}

jaco_manipulation::PlanAndMoveArmGoal KinectGoal::goal() const {
  return PoseGoal::goal();
}

//void KinectGoal::adjustPoseToCenterOfObject(const goal_input::BoundingBox &bounding_box) {
//  // TODO BOUNDING BOX CALCULATED BY TOP RIGHT CORNER IN COORDINATE SYSTEM. CHANGE AFTER FEEDBACK WITH ANDREAS
//  goal_.pose_goal.pose.position.x = bounding_box.x;
//  goal_.pose_goal.pose.position.y = bounding_box.y;
//  goal_.pose_goal.pose.position.z = bounding_box.z;
//
//  const double width_adj = bounding_box.width * 0.5;
//  const double length_adj = bounding_box.length * 0.5;
//
//  goal_.pose_goal.pose.position.x += (bounding_box.x >= 0.0) ? width_adj : -width_adj;
//  goal_.pose_goal.pose.position.y += (bounding_box.y >= 0.0) ? -length_adj : length_adj;
//
//  ROS_INFO("Box Fix : (%f %f %f) -> (%f %f %f)",
//                                      bounding_box.x,
//                                      bounding_box.y,
//                                      bounding_box.z,
//                                      goal_.pose_goal.pose.position.x,
//                                      goal_.pose_goal.pose.position.y,
//                                      goal_.pose_goal.pose.position.z);
//}

void KinectGoal::adjustPose(jaco_manipulation::grasps::GraspType grasp,
                            const jaco_manipulation::goals::goal_input::BoundingBox &box) {
  grasp_orientation_generator_.adjustPose(goal_.pose_goal, box, grasp);
}

std::string KinectGoal::requestedOrientation() {
  using jaco_manipulation::grasps::GraspType;
  switch (requested_grasp_) {
    case GraspType::TOP_GRASP:
      return "Top Orientation";
    case GraspType::FRONT_GRASP:
      return "Frontal Orientation";
    case GraspType::LEFT_GRASP:
      return "Left Orientation";
    case GraspType::RIGHT_GRASP:
      return "Right Orientation";
    default:
      return "Any Orientation";
  }
}
