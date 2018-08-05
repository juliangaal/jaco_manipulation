//
// Created by julian on 05.08.18.
//

#ifndef PROJECT_CLIENT_H
#define PROJECT_CLIENT_H

#include <actionlib/client/simple_action_client.h>
#include <jaco_manipulation/PlanAndMoveArmAction.h>
#include <jaco_manipulation/server/jaco_manipulation.h>

#include "joint_goal.h"
#include "move_it_goal.h"
#include "pose_goal.h"

using PamClient = actionlib::SimpleActionClient<jaco_manipulation::PlanAndMoveArmAction>;

namespace Moveit {

class Client {
 public:
  explicit Client();

  ~Client();

  template<typename T>
  void execute(const T &goal);
 private:
  PamClient client_;
};
}

#endif //PROJECT_CLIENT_H
