//
// Created by julian on 04.08.18.
//

#ifndef PROJECT_MOVEITGOAL_H
#define PROJECT_MOVEITGOAL_H

#include "goal.h"

using std::string;

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
  explicit MoveItGoal(const string &name);

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

#endif //PROJECT_MOVEITGOAL_H
