//
// Created by chitt on 8/9/18.
//

#ifndef PROJECT_GRASP_H
#define PROJECT_GRASP_H

namespace jaco_manipulation {
namespace client {
namespace grasps {

class GraspOrientation {
 protected:
  constexpr GraspOrientation(double rot_x, double rot_y, double rot_z, double min_height)
      : rotationX_(rot_x), rotationY_(rot_y), rotationZ_(rot_z), min_height_(min_height) {}
 public:
  virtual ~GraspOrientation() = default;
  const double rotationX_;
  const double rotationY_;
  const double rotationZ_;
  const double min_height_;
};
} // namespace grasps
} // namespace client
} // namespace jaco_manipulation

#endif //PROJECT_GRASP_H
