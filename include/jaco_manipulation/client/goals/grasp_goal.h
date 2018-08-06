//
// Created by chitt on 8/6/18.
//

#ifndef PROJECT_GRASPGOAL_HPP
#define PROJECT_GRASPGOAL_HPP
#include "goal.h"

namespace jaco_manipulation {
namespace client {
namespace goals {

class GraspGoal : public Goal {
 public:
  struct GraspPose {
    double x;
    double y;
    double z;
    double rotation;
  };

  GraspGoal() = delete;
  GraspGoal(const GraspPose &grasp_pose_goal);
  virtual ~GraspGoal() = default;

  virtual jaco_manipulation::PlanAndMoveArmGoal getGoal() const final;
};

} // namespace jaco_manipulation
} // namespace client
} // namespace goals

#endif //PROJECT_GRASPGOAL_HPP
