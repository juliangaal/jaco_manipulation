//
// Created by julian on 06.08.18.
//

#ifndef PROJECT_DROP_GOAL_H
#define PROJECT_DROP_GOAL_H

#include "kinect_goal.h"

namespace jaco_manipulation {
namespace goals {

/*!
 * DropGoal
 * Represent a severely limited pose for dropping objects
 * Limited in height, and rotation around x and y axis
 */
class DropGoal: public KinectGoal {
 public:

  /**
   * Constructor
   * @param drop_pose_goal drop pose goal
   * @param description descritpion with additional info
   */
  explicit DropGoal(const goal_input::LimitedPose &drop_pose_goal,
                    const std::string &description = "drop goal");

  /**
   * Constructor
   * @param bounding_box_goal bounding box to drop something at
   * @param description descritpion with additional info
   */
  explicit DropGoal(const goal_input::BoundingBox &bounding_box_goal,
                    const std::string &description = "drop box goal");

  /**
   * default destructor
   */
  ~DropGoal() final = default;

  /**
   * get created goal
   */
  jaco_manipulation::PlanAndMoveArmGoal goal() const final;
};

} // namespace goals
} // namespace jaco_manipulation

#endif //PROJECT_DROP_GOAL_H
