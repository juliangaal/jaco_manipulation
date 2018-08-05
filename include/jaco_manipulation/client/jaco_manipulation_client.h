//
// Created by julian on 05.08.18.
//

#ifndef PROJECT_JACOMANIPULATIONCLIENT_H
#define PROJECT_JACOMANIPULATIONCLIENT_H

#define ROS_SUCCESS(x) ROS_INFO_STREAM("\033[32m" << (x) << "\033[00m")

#include <actionlib/client/simple_action_client.h>
#include <jaco_manipulation/PlanAndMoveArmAction.h>

#include <jaco_manipulation/client/goals/goal.h>
#include <jaco_manipulation/client/goals/joint_goal.h>
#include <jaco_manipulation/client/goals/move_it_goal.h>
#include <jaco_manipulation/client/goals/pose_goal.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

using sensor_msgs::JointState;
using geometry_msgs::PoseStamped;
using PamClient = actionlib::SimpleActionClient<jaco_manipulation::PlanAndMoveArmAction>;

/**
 * JacoManipulationClient
 * Simple but convenient wrapper around ROS action server SimpleActionClient, tuned for Jaco Robot Arm
 * Joint-, Pose- and MoveItGoals can be sent to the SimpleActionServer.
 * All goals are defined in /include/jaco_manipulation/client/goals
 */
class JacoManipulationClient {
 public:
  /**
   * default constructor
   */
  JacoManipulationClient();

  /**
   * default destructor
   */
  ~JacoManipulationClient() = default;

  /**
   * Move to MoveIt goal defined in jaco.srdf
   * @param goal name of goal
   */
  void moveTo(const string &goal);

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
  /**
   * PlanAndMoveArmAction ROS action server client
   */
  PamClient client_;

  /**
   * Executes goal: sends to action server and gets result
   * @param goal MoveItGoal, JointGoal, or PoseGoal
   */
  void execute(const Goal& goal);

};

#endif //PROJECT_JACOMANIPULATIONCLIENT_H
