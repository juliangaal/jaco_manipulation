//
// Created by chitt on 8/9/18.
//

#ifndef PROJECT_GRASP_ORIENTATION_GENERATOR_H
#define PROJECT_GRASP_ORIENTATION_GENERATOR_H

#include <geometry_msgs/PoseStamped.h>

namespace jaco_manipulation {
namespace grasps {

enum GraspType { TOP_GRASP, FRONT_GRASP, LEFT_GRASP, RIGHT_GRASP };

class GraspPoseGenerator {
 public:
  constexpr GraspPoseGenerator() = default;
  void adjustOrientation(geometry_msgs::PoseStamped &pose, const GraspType type);
 private:
  void adjustTopGraspOrientation(geometry_msgs::PoseStamped &pose);
  void adjustFrontGraspOrientation(geometry_msgs::PoseStamped &pose);
  constexpr static double min_height_top_grasp = 0.175026;
  constexpr static double min_height_front_grasp = 0.1;
};

} // namespace grasps
} // namespace jaco_manipulation

#endif //PROJECT_GRASP_ORIENTATION_GENERATOR_H
