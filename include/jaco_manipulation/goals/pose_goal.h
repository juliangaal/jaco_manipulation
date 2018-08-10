//
// Created by julian on 04.08.18.
//

#ifndef PROJECT_POSEGOAL_H
#define PROJECT_POSEGOAL_H

#include "goal.h"

namespace jaco_manipulation {
namespace goals {

/*!
 * PoseGoal
 * class to define a pose goal for MoveIt! execution
*/
class PoseGoal : public Goal {
 public:

  /**
   * deleted constructor
   * @param name future goal_type
   */
  PoseGoal(const std::string &name) = delete;

  /**
   * constructor
   * @param pose geometry_msgs::PoseStamped to move to
   * @param description description to make client console output more clear
   */
  explicit PoseGoal(const geometry_msgs::PoseStamped &pose, const std::string &description="pose goal");

  /**
   * virtual default destructor
   */
  virtual ~PoseGoal() override = default;

  /**
   * get goal that was created
   * @return jaco_manipulation::PlanAndMoveArmGoal
 */
  virtual jaco_manipulation::PlanAndMoveArmGoal goal() const override;

 protected:

  /**
   * protected default constructor
  */
  PoseGoal() = default;

  /**
   * Adjusts height for poses that are too low
  */
  void adjustHeight();

  /**
   * Minimum height for pose
  */
  constexpr static double min_height_ = 0.175026;
};

} // namespace goals
} // namespace jaco_manipulation

#endif //PROJECT_POSEGOAL_H
