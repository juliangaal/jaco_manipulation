//
// Created by chitt on 8/9/18.
//

#ifndef PROJECT_FRONT_GRASP_ORIENTATION_H
#define PROJECT_FRONT_GRASP_ORIENTATION_H

#include <jaco_manipulation/client/grasps/grasp_orientation.h>

namespace jaco_manipulation {
namespace client {
namespace grasps {

struct FrontGraspOrientation : public GraspOrientation {
  constexpr FrontGraspOrientation() : GraspOrientation(1, -1, 1, 0.15) {}
  ~FrontGraspOrientation() final = default;
};
} // namespace grasps
} // namespace client
} // namespace jaco_manipulation

#endif //PROJECT_FRONT_GRASP_ORIENTATION_H
