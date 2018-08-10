//
// Created by julian on 06.08.18.
//

#include <jaco_manipulation/client/goals/objects/drop_goal.h>
#include <ros/console.h>

using namespace jaco_manipulation::client::goals::objects;

DropGoal::DropGoal(const kinect_goal_definitions::LimitedPose &drop_pose_goal, const std::string &description)
: KinectGoal(drop_pose_goal, jaco_manipulation::client::grasps::GraspType::TOP_GRASP, description) {
  goal_.goal_type = "drop_pose";

  goal_.pose_goal.pose.position.z += dropping_offset_;

  ROS_INFO_STREAM("Attempt : Move to " << info());
}

DropGoal::DropGoal(const kinect_goal_definitions::BoundingBox &bounding_box_goal, const std::string &description)
: KinectGoal(bounding_box_goal, jaco_manipulation::client::grasps::GraspType::TOP_GRASP, description) {
  goal_.goal_type = "drop_pose";

  adjustHeight(bounding_box_goal);

  ROS_INFO_STREAM("Attempt : Move to " << info());
}

jaco_manipulation::PlanAndMoveArmGoal DropGoal::goal() const {
  return KinectGoal::goal();
}

void DropGoal::adjustHeight(const jaco_manipulation::client::goals::kinect_goal_definitions::BoundingBox &bounding_box) {
  double height_adj = 0.0;
  // TODO this works even though it might seem to low. Jaco's, e.g. h=17cm is not real world height of 17cm, ASK CHITT
  // works even without dropping offset

  // if goal_.pose.pose.position.z has been adjusted for top grasp, now we have to add the offset for the bounding box
  if (bounding_box.height > goal_.pose_goal.pose.position.z)
    height_adj = std::fabs(bounding_box.height - goal_.pose_goal.pose.position.z);

  height_adj += dropping_offset_;
  // TODO may be have to be adjusted once object is added to planning scene

  goal_.pose_goal.pose.position.z += height_adj;
}
