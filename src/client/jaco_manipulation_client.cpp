//
// Created by julian on 05.08.18.
//

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
}


void JacoManipulationClient::graspAt(const goals::goal_input::BoundingBox &bounding_box_goal,
                                     const std::string &description) {
  tryDifferentGraspPoses(bounding_box_goal, description);
}

void JacoManipulationClient::dropAt(const goals::goal_input::LimitedPose &drop_pose_goal,
                                    const std::string &description) {
  goals::DropGoal goal(drop_pose_goal, description);
  execute(goal);
}

void JacoManipulationClient::dropAt(const goals::goal_input::BoundingBox &bounding_box_goal,
                                    const std::string &description) {
  goals::DropGoal goal(bounding_box_goal, description);
  execute(goal);
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
void JacoManipulationClient::tryDifferentGraspPoses(const T &goal_type, const std::string &description) {
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
