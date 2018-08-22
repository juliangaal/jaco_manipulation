//
// Created by chitt on 8/9/18.
//

#ifndef PROJECT_GRASP_ORIENTATION_GENERATOR_H
#define PROJECT_GRASP_ORIENTATION_GENERATOR_H

#include <geometry_msgs/PoseStamped.h>
#include <jaco_manipulation/goals/goal_input.h>
#include <jaco_manipulation/BoundingBox.h>
#include <tf/transform_listener.h>
#include <ros/node_handle.h>

namespace jaco_manipulation {
namespace grasps {

enum GraspType { TOP_GRASP, TOP_DROP, FRONT_GRASP, LEFT_GRASP, RIGHT_GRASP };

class GraspPoseGenerator {
 public:
  GraspPoseGenerator();
  void adjustPose(geometry_msgs::PoseStamped &pose,
                  const jaco_manipulation::BoundingBox &box,
                  const GraspType type);
 private:
  void adjustHeightForTopPose(geometry_msgs::PoseStamped &pose,
                                     const jaco_manipulation::BoundingBox &box);
  void adjustHeightForTopDropPose(geometry_msgs::PoseStamped &pose,
                                     const jaco_manipulation::BoundingBox &box);
  void adjustHeightForFrontPose(geometry_msgs::PoseStamped &pose,
                                                    const jaco_manipulation::BoundingBox &box);
  void adjustPosition(geometry_msgs::PoseStamped &pose,
                      const jaco_manipulation::BoundingBox &box,
                      const GraspType type);
  void adjustToTopOrientation(geometry_msgs::PoseStamped &pose);
  void adjustToFrontOrientation(geometry_msgs::PoseStamped &pose);
  void adjustPoseForFrontPose(geometry_msgs::PoseStamped &pose);

  void transformGoalIntoRobotFrame(geometry_msgs::PoseStamped &pose,
                                   const jaco_manipulation::BoundingBox &box);

  ros::NodeHandle n_;
  tf::TransformListener tf_listener_;
  constexpr static double min_height_top_grasp = 0.175026;
  constexpr static double min_height_front_grasp = 0.011;
  constexpr static double drop_offset_ = 0.03;
  constexpr static double stack_offset_ = 0.01;
  constexpr static double grasp_offset_ = 0.145;
};

} // namespace grasps
} // namespace jaco_manipulation

#endif //PROJECT_GRASP_ORIENTATION_GENERATOR_H
