/*
  Copyright (C) 2018  Julian Gaal
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#ifndef PROJECT_JACOMANIPULATIONCLIENT_H
#define PROJECT_JACOMANIPULATIONCLIENT_H

#define ROS_SUCCESS(x) ROS_INFO_STREAM("\033[32m" << x << "\033[00m")

#include <actionlib/client/simple_action_client.h>
#include <jaco_manipulation/PlanAndMoveArmAction.h>
#include <jaco_manipulation/BoundingBox.h>

#include <jaco_manipulation/goals/goal.h>
#include <jaco_manipulation/goals/grasp_goal.h>
#include <jaco_manipulation/goals/joint_goal.h>
#include <jaco_manipulation/goals/move_it_goal.h>
#include <jaco_manipulation/goals/pose_goal.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

namespace jaco_manipulation {
namespace client {

/*!
 * JacoManipulationClient
 * Simple but convenient wrapper around ROS action server SimpleActionClient, tuned for Jaco Robot Arm
 * Joint-, Pose- and MoveItGoals can be sent to the SimpleActionServer.
 * All goals are defined in /include/jaco_manipulation/goals/
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
  void graspAt(const goals::goal_input::LimitedPose &grasp_pose_goal,
               const std::string &description = "grasp goal");

  /**
    * Move to grasp goal according to bounding box
    * @param bounding_box_goal bounding box to drop something at
    * @param description description of move
  */
  void graspAt(const jaco_manipulation::BoundingBox &bounding_box_goal,
               const std::string &description = "grasp box goal");

  /**
   * Move to drop goal
   * @param goal drop pose goal
   * @param description description of move
  */
  void dropAt(const goals::goal_input::LimitedPose &drop_pose_goal,
              const std::string &description = "drop goal");

  /**
   * Move to drop goal according to bounding box
   * @param bounding_box_goal bounding box to drop something at
   * @param description description of move
  */
  void dropAt(const jaco_manipulation::BoundingBox &bounding_box_goal,
              const std::string &description = "drop box goal");

 private:
  /**
   * PlanAndMoveArmAction ROS action server client
   */
  SimpleActionClient client_;

  /**
   * Executes goal: sends to action server and gets result
   * @param goal MoveItGoal, JointGoal, or PoseGoal
   * @param show_result_information whether or not to show grasp information
   */
  bool execute(const goals::Goal& goal, bool show_result_information=true);

  /**
   *
   */
  template <typename T>
  void tryDifferentGraspPoses(T &goal_type,
                         const std::string &description);
  };

} // namespace client
} // namespace jaco_manipulation

#endif //PROJECT_JACOMANIPULATIONCLIENT_H
