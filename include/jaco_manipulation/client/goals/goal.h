//
// Created by julian on 04.08.18.
//

#ifndef PROJECT_GOAL_H
#define PROJECT_GOAL_H

#include <actionlib/client/simple_action_client.h>
#include <jaco_manipulation/PlanAndMoveArmAction.h>
#include <jaco_manipulation/server/jaco_manipulation_server.h>
#include <string>
#include "goal.h"

using std::string;

using PamClient = actionlib::SimpleActionClient<jaco_manipulation::PlanAndMoveArmAction>;

class Goal {
 public:
  Goal();
  Goal(const string &name) = delete;
  virtual ~Goal() = default;

  virtual jaco_manipulation::PlanAndMoveArmGoal getGoal() const;

 protected:
  jaco_manipulation::PlanAndMoveArmGoal goal;
  const string planning_frame;
};

#endif //PROJECT_GOAL_H
