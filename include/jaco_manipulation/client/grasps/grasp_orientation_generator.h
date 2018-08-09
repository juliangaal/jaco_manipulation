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
  void adjustOrientation(const GraspType type);
  void adjustOrientation(const GraspType type, geometry_msgs::PoseStamped &pose);
  void adjustOrientation(const FrontGraspOrientation &grasp, geometry_msgs::PoseStamped &pose);
};

} // namespace grasps
} // namespace client
} // namespace jaco_manipulation

#endif //PROJECT_GRASP_ORIENTATION_GENERATOR_H
