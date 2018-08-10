//
// Created by chitt on 8/9/18.
//

#ifndef PROJECT_GRASP_ORIENTATION_GENERATOR_H
#define PROJECT_GRASP_ORIENTATION_GENERATOR_H

#include <geometry_msgs/PoseStamped.h>
#include <jaco_manipulation/client/grasps/grasp_orientation.h>
#include <jaco_manipulation/client/grasps/top_grasp_orientation.h>
#include <jaco_manipulation/client/grasps/front_grasp_orientation.h>

namespace jaco_manipulation {
namespace client {
namespace grasps {

enum GraspType { TOP_GRASP, FRONT_GRASP, LEFT_GRASP, RIGHT_GRASP };

class GraspOrientationGenerator {
 public:
  constexpr GraspOrientationGenerator() = default;
  void adjustOrientation(geometry_msgs::PoseStamped &pose, const GraspType type);
 private:
  void adjustTopGraspOrientation(geometry_msgs::PoseStamped &pose);
  void adjustFrontGraspOrientation(geometry_msgs::PoseStamped &pose);
  constexpr static double min_height_top_grasp = 0.175026;
  constexpr static double min_height_front_grasp = 0.1;
};

} // namespace grasps
} // namespace client
} // namespace jaco_manipulation

#endif //PROJECT_GRASP_ORIENTATION_GENERATOR_H
