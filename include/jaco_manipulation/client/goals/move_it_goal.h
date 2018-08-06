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
   * @param name future goal_type
   */
  explicit MoveItGoal(const std::string &name);

  /**
   * virtual default destructor
   */
  virtual ~MoveItGoal() = default;

  /**
    * get goal that was created
    * @return jaco_manipulation::PlanAndMoveArmGoal
  */
  virtual jaco_manipulation::PlanAndMoveArmGoal getGoal() const final;

};

} // namespace jaco_manipulation
} // namespace client
} // namespace goals

#endif //PROJECT_MOVEITGOAL_H
