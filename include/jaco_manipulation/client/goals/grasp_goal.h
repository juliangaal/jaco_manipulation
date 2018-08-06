//
// Created by chitt on 8/6/18.
//

#ifndef PROJECT_GRASPGOAL_HPP
#define PROJECT_GRASPGOAL_HPP
#include "goal.h"

namespace jaco_manipulation {
namespace client {
namespace goals {

/**
 * GraspGoal
 * Represent a severely limited pose for grasping objects
 * Limited in height, and rotation around x and y axis
 */
class GraspGoal : public Goal {
 public:

  /**
   * additional layer of security: GraspPose allows only to change properties on pose that are not dangerous
   */
  struct GraspPose {
    double x;
    double y;
    double z;
    double rotation;
  };

  /**
   * deleted default constructor
   */
  GraspGoal() = delete;

  /**
   * Constructor
   * @param grasp_pose_goal grasp pose goal
   * @param description descritpion with additional info
   */
  GraspGoal(const GraspPose &grasp_pose_goal, const std::string &description="grasp goal");

  /**
   * default destructor
   */
  ~GraspGoal() final = default;

  jaco_manipulation::PlanAndMoveArmGoal getGoal() const final;

  /**
   * get description of goal
   * @return std::string description
 */
  const std::string& getDescription() const final;
};

} // namespace jaco_manipulation
} // namespace client
} // namespace goals

#endif //PROJECT_GRASPGOAL_HPP
