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

KinectGoal::KinectGoal(const jaco_manipulation::BoundingBox &bounding_box_goal,
                       jaco_manipulation::grasps::GraspType grasp,
                       const std::string &description)
: requested_grasp_(grasp) {
  description_ = description + " \"" + bounding_box_goal.description + "\"";

  goal_.pose_goal.header.frame_id = planning_frame_;
  goal_.goal_type = "goal";

  adjustPose(grasp, bounding_box_goal);
}

jaco_manipulation::PlanAndMoveArmGoal KinectGoal::goal() const {
  return PoseGoal::goal();
}

void KinectGoal::adjustPose(jaco_manipulation::grasps::GraspType grasp,
                            const jaco_manipulation::BoundingBox &box) {
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
