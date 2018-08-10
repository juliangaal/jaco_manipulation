//
// Created by julian on 8/6/18.
//

#include <jaco_manipulation/client/goals/objects/grasp_goal.h>
#include <ros/console.h>

using namespace jaco_manipulation::client::goals::objects;

GraspGoal::GraspGoal(const kinect_goal_definitions::LimitedPose &grasp_pose_goal,
                     jaco_manipulation::client::grasps::GraspType grasp,
                     const std::string &description)

: KinectGoal(grasp_pose_goal, grasp, description) {
  goal_.goal_type = "grasp_pose";

  ROS_INFO_STREAM("Attempt : Move to " << info());
}

GraspGoal::GraspGoal(const kinect_goal_definitions::BoundingBox &bounding_box_goal,
                     jaco_manipulation::client::grasps::GraspType grasp,
                     const std::string &description)
: KinectGoal(bounding_box_goal, grasp, description) {
  goal_.goal_type = "grasp_pose";

  if (grasp == jaco_manipulation::client::grasps::GraspType::TOP_GRASP)
    adjustHeight(bounding_box_goal);

  ROS_INFO_STREAM("Attempt : Move to " << info());
}

jaco_manipulation::PlanAndMoveArmGoal GraspGoal::goal() const {
  return KinectGoal::goal();
}

void GraspGoal::adjustHeight(const jaco_manipulation::client::goals::kinect_goal_definitions::BoundingBox &bounding_box) {
  double height_adj = 0.0;

  if (bounding_box.height > goal_.pose_goal.pose.position.z)
    height_adj = std::fabs(bounding_box.height - goal_.pose_goal.pose.position.z);

  goal_.pose_goal.pose.position.z += height_adj;
}
