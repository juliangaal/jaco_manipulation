//
// Created by chitt on 8/9/18.
//

#ifndef PROJECT_GRASP_ORIENTATION_GENERATOR_H
#define PROJECT_GRASP_ORIENTATION_GENERATOR_H

#include <geometry_msgs/PoseStamped.h>
#include <jaco_manipulation/goals/goal_input.h>

namespace jaco_manipulation {
namespace grasps {

enum GraspType { TOP_GRASP, TOP_DROP, FRONT_GRASP, LEFT_GRASP, RIGHT_GRASP };

class GraspPoseGenerator {
 public:
  constexpr GraspPoseGenerator() = default;
  void adjustPose(geometry_msgs::PoseStamped &pose,
                  const jaco_manipulation::goals::goal_input::BoundingBox &box,
                  const GraspType type);
 private:
  void adjustPosition(geometry_msgs::PoseStamped &pose,
                      const jaco_manipulation::goals::goal_input::BoundingBox &box,
                      const GraspType type);
  void adjustTopGraspOrientation(geometry_msgs::PoseStamped &pose);
  void adjustFrontGraspOrientation(geometry_msgs::PoseStamped &pose);
  constexpr static double min_height_top_grasp = 0.175026;
  constexpr static double min_height_front_grasp = 0.1;
};

} // namespace grasps
} // namespace jaco_manipulation

#endif //PROJECT_GRASP_ORIENTATION_GENERATOR_H
