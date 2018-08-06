//
// Created by julian on 04.08.18.
//

#ifndef PROJECT_MOVEITGOAL_H
#define PROJECT_MOVEITGOAL_H

#include "goal.h"

namespace jaco_manipulation {
namespace client {
namespace goals {

class MoveItGoal : public Goal {
 public:
  /**
   * deleted default constructor
  */
  MoveItGoal() = delete;

  /**
   * constructor
   * @param goal_name name of goal_type in moveit config
   * @param description description to make client console output more clear
  */
  explicit MoveItGoal(const std::string &goal_name, const std::string &description="movit config file goal");

  /**
   * virtual default destructor
   */
  ~MoveItGoal() final = default;

  /**
    * get goal that was created
    * @return jaco_manipulation::PlanAndMoveArmGoal
  */
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

#endif //PROJECT_MOVEITGOAL_H
