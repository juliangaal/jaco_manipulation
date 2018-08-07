//
// Created by julian on 05.08.18.
//

#ifndef PROJECT_JACOMANIPULATIONCLIENT_H
#define PROJECT_JACOMANIPULATIONCLIENT_H

#define ROS_SUCCESS(x) ROS_INFO_STREAM("\033[32m" << x << "\033[00m")

#include <actionlib/client/simple_action_client.h>
#include <jaco_manipulation/PlanAndMoveArmAction.h>

#include <jaco_manipulation/client/goals/goal.h>
#include <jaco_manipulation/client/goals/objects/grasp_goal.h>
#include <jaco_manipulation/client/goals/joint_goal.h>
#include <jaco_manipulation/client/goals/move_it_goal.h>
#include <jaco_manipulation/client/goals/pose_goal.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

namespace jaco_manipulation {
namespace client {

/*!
 * JacoManipulationClient
 * Simple but convenient wrapper around ROS action server SimpleActionClient, tuned for Jaco Robot Arm
 * Joint-, Pose- and MoveItGoals can be sent to the SimpleActionServer.
 * All goals are defined in /include/jaco_manipulation/client/goals
 */
class JacoManipulationClient {
 public:

  using SimpleActionClient = actionlib::SimpleActionClient<jaco_manipulation::PlanAndMoveArmAction>;

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
   * @param description description of move
   */
  void moveTo(const std::string &goal, const std::string &description="MoveIt! config goal");

  /**
   * Move to Pose goal
   * @param goal pose goal
   * @param description description of move
   */
  void moveTo(const geometry_msgs::PoseStamped &goal, const std::string &description="pose goal");

  /**
   * Move to joint goal
   * @param goal joint goal
   * @param description description of move
   */
  void moveTo(const sensor_msgs::JointState &goal, const std::string &description="joint state goal");

  /**
    * Move to grasp goal
    * @param goal grasp pose goal
    * @param description description of move
    */
  void grasp(const goals::object_helper::LimitedPose &grasp_pose_goal, const std::string &description="grasp goal");

  /**
  * Move to drop goal
  * @param goal drop pose goal
  * @param description description of move
  */
  void drop(const goals::object_helper::LimitedPose &drop_pose_goal, const std::string &description="drop goal");

 private:
  /**
   * PlanAndMoveArmAction ROS action server client
   */
  SimpleActionClient client_;

  /**
   * Executes goal: sends to action server and gets result
   * @param goal MoveItGoal, JointGoal, or PoseGoal
   */
  void execute(const goals::Goal& goal);

  /**
   * Convenience function to get all relevant info from goal
   * @param goal goal that will be moved to
   * @return information about move goal
   */
  const std::string info(const goals::Goal &goal_wrapper) const;
};

} // namespace client
} // namespace jaco_manipulation

#endif //PROJECT_JACOMANIPULATIONCLIENT_H
