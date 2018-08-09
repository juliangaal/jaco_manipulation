//
// Created by chitt on 8/9/18.
//

#ifndef PROJECT_TOP_GRASP_H
#define PROJECT_TOP_GRASP_H

#include <jaco_manipulation/client/grasps/grasp_orientation.h>

namespace jaco_manipulation {
namespace client {
namespace grasps {

struct TopGraspOrientation : public GraspOrientation {
  constexpr TopGraspOrientation() : GraspOrientation(1, -1, 1, 0.175026) {}
  ~TopGraspOrientation() final = default;
};
} // namespace grasps
} // namespace client
} // namespace jaco_manipulation

#endif //PROJECT_TOP_GRASP_H
