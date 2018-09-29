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

#include <jaco_manipulation/client/jaco_manipulation_client.h>
#include <jaco_manipulation/goals/grasp_goal.h>
#include <jaco_manipulation/goals/drop_goal.h>
#include <jaco_manipulation/grasps/grasp_pose_generator.h>
#include <tf/tf.h>

using namespace jaco_manipulation::client;

JacoManipulationClient::JacoManipulationClient() : client_("plan_and_move_arm", true) {
  client_.waitForServer();
}

void JacoManipulationClient::moveTo(const std::string &moveit_goal, const std::string &description) {
  goals::MoveItGoal goal(moveit_goal, description);
  execute(goal);
}

void JacoManipulationClient::moveTo(const geometry_msgs::PoseStamped &pose_goal, const std::string &description) {
  goals::PoseGoal goal(pose_goal, description);
  execute(goal);
}

void JacoManipulationClient::moveTo(const sensor_msgs::JointState &joint_goal, const std::string &description) {
  goals::JointGoal goal(joint_goal.position, description);
  execute(goal);
}

void JacoManipulationClient::graspAt(const goals::goal_input::LimitedPose &grasp_pose_goal,
                                     const std::string &description) {
  tryDifferentGraspPoses(grasp_pose_goal, description);
  moveTo("post_grasp");
}


void JacoManipulationClient::graspAt(const jaco_manipulation::BoundingBox &bounding_box_goal,
                                     const std::string &description) {
  tryDifferentGraspPoses(bounding_box_goal, description);
  moveTo("post_grasp");
}

void JacoManipulationClient::dropAt(const goals::goal_input::LimitedPose &drop_pose_goal,
                                    const std::string &description) {
  goals::DropGoal goal(drop_pose_goal, description);
  execute(goal);
  moveTo("home");
}

void JacoManipulationClient::dropAt(const jaco_manipulation::BoundingBox &bounding_box_goal,
                                    const std::string &description) {
  goals::DropGoal goal(bounding_box_goal, description);
  execute(goal);
  moveTo("home");
}

bool JacoManipulationClient::execute(const goals::Goal &goal_input, bool show_result_information) {
  const auto& goal = goal_input.goal();

  client_.sendGoal(goal);
  client_.waitForResult();

  if (client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    if (show_result_information)
      ROS_SUCCESS("Status  : Move to " << goal_input.info() << " succeeded.\n");
    return true;
  } else {
    if (show_result_information)
      ROS_ERROR_STREAM("Status  : Move to " << goal_input.info() << " failed.\n");
    return false;
  }
}

template <typename T>
void JacoManipulationClient::tryDifferentGraspPoses(T &goal_type, const std::string &description) {
  {
    goals::GraspGoal goal(goal_type,
                          jaco_manipulation::grasps::GraspType::TOP_GRASP,
                          description);
    if (execute(goal, false)) {
      ROS_SUCCESS("Status  : Move to " << goal.info() << " with " << goal.requestedOrientation() << " succeeded.\n");
      return;
    } else {
      ROS_ERROR_STREAM("Status  : Move to " << goal.info() << " with " << goal.requestedOrientation() << " failed.\n");
    }
  }
  {
    goals::GraspGoal goal(goal_type,
                          jaco_manipulation::grasps::GraspType::FRONT_GRASP,
                          description);
    if (execute(goal, false)) {
      ROS_SUCCESS("Status  : Move to " << goal.info() << " with " << goal.requestedOrientation() << "succeeded.\n");
      return;
    } else {
      ROS_ERROR_STREAM("Status  : Move to " << goal.info() << " with " << goal.requestedOrientation() << "failed.\n");
    }
  }
}

void JacoManipulationClient::updatePlanningScene(const jaco_manipulation::BoundingBox &box) {
  jaco_manipulation::PlanAndMoveArmGoal goal;
  goal.pose_goal.header.frame_id = "root";
  goal.goal_type = "add_obstacle";
  goal.bounding_box = box;

  client_.sendGoal(goal);
  client_.waitForResult();
}

void JacoManipulationClient::wipePlanningScene() {
  jaco_manipulation::PlanAndMoveArmGoal goal;
  goal.pose_goal.header.frame_id = "root";
  goal.goal_type = "wipe_kinect_obstacles";

  client_.sendGoal(goal);
  client_.waitForResult();
}
