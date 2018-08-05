//
// Created by julian on 05.08.18.
//

#ifndef PROJECT_JACOMANIPULATIONCLIENT_H
#define PROJECT_JACOMANIPULATIONCLIENT_H

#include <actionlib/client/simple_action_client.h>
#include <jaco_manipulation/PlanAndMoveArmAction.h>

#include <jaco_manipulation/client/goals/goal.h>
#include <jaco_manipulation/client/goals/joint_goal.h>
#include <jaco_manipulation/client/goals/move_it_goal.h>
#include <jaco_manipulation/client/goals/pose_goal.h>

using PamClient = actionlib::SimpleActionClient<jaco_manipulation::PlanAndMoveArmAction>;

class JacoManipulationClient {
 public:
  JacoManipulationClient();

  ~JacoManipulationClient() = default;

  /**
   * Move to MoveIt goal defined in jaco.srdf
   * @param goal name of goal
   */
  void moveTo(string goal);

  /**
   * Move to Pose goal
   * @param goal pose goal
   */
  void moveTo(const PoseStamped &goal);

  /**
   * Move to joint goal
   * @param goal joint goal
   */
  void moveTo(const JointState &goal);

 private:
  PamClient client_;

  void execute(const Goal& goal);

};

#endif //PROJECT_JACOMANIPULATIONCLIENT_H
