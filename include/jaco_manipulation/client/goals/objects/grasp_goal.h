//
// Created by chitt on 8/6/18.
//

#ifndef PROJECT_GRASPGOAL_HPP
#define PROJECT_GRASPGOAL_HPP
#include "jaco_manipulation/client/goals/objects/object_goal.h"

namespace jaco_manipulation {
namespace client {
namespace goals {
namespace objects {

/*!
 * GraspGoal
 * Represent a severely limited pose for grasping objects
 * Limited in height, and rotation around x and y axis
 */
class GraspGoal: public KinectGoal {
 public:

  /**
   * deleted default constructor
  */
  GraspGoal() = delete;

  /**
   * Constructor
   * @param grasp_pose_goal grasp pose goal
   * @param description descritpion with additional info
   */
  explicit GraspGoal(const kinect_goal_definitions::LimitedPose &grasp_pose_goal, const std::string &description = "grasp goal");

  /**
   * Constructor
   * @param bounding_box_goal bounding box to drop something at
   * @param description descritpion with additional info
   */
  explicit GraspGoal(const kinect_goal_definitions::BoundingBox &bounding_box_goal, const std::string &description = "grasp box goal");

  /**
   * default destructor
   */
  ~GraspGoal() final = default;

  /**
   * get created goal
   */
  jaco_manipulation::PlanAndMoveArmGoal goal() const final;
};

} // namespace objects
} // namespace goals
} // namespace client
} // namespace jaco_manipulation

#endif //PROJECT_GRASPGOAL_HPP
