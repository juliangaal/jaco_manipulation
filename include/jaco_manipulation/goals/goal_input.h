//
// Created by julian on 10.08.18.
//

#ifndef PROJECT_GOAL_INPUT_H
#define PROJECT_GOAL_INPUT_H

namespace jaco_manipulation {
namespace goals {
namespace goal_input {

/**
 * Object to grasp, defined by bounding box
 */
struct BoundingBox {
  std::string description;
  std::string type;
  double x;
  double y;
  double z;

  double height;
  double width;
  double length;
};

/**
 * additional layer of security: grasp_helper::GraspPose allows only to change properties on pose that are not dangerous
 */
struct LimitedPose {
  double x;
  double y;
  double z;
  double rotation;
};
} // goal_input
} // namespace goals
} // namespace jaco_manipulation

#endif //PROJECT_GOAL_INPUT_H
