//
// Created by julian on 06.08.18.
//

#ifndef PROJECT_DROP_GOAL_H
#define PROJECT_DROP_GOAL_H

#include "jaco_manipulation/client/goals/objects/grasp_goal.h"

namespace jaco_manipulation {
namespace client {
namespace goals {

/*!
 * DropGoal
 * Represent a severely limited pose for dropping objects
 * Limited in height, and rotation around x and y axis
 */
class DropGoal : public GraspGoal {
 public:

  /**
   * Constructor
   * @param drop_pose_goal drop pose goal
   * @param description descritpion with additional info
   */
  explicit DropGoal(const grasp_helper::GraspPose &drop_pose_goal, const std::string &description="drop goal");


  explicit DropGoal(const grasp_helper::Object &object_goal, const std::string &description="drop box goal");

  /**
   * default destructor
   */
  ~DropGoal() final = default;

  /**
   * get created goal
   */
  jaco_manipulation::PlanAndMoveArmGoal getGoal() const final;

  /**
   * get description of goal
   * @return std::string description
 */
  const std::string& getDescription() const final;
};

} // namespace goals
} // namespace client
} // namespace jaco_manipulation

#endif //PROJECT_DROP_GOAL_H
