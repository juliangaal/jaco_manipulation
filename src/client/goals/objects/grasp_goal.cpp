//
// Created by julian on 8/6/18.
//

#include <jaco_manipulation/client/goals/objects/grasp_goal.h>
#include <ros/console.h>

using namespace jaco_manipulation::client::goals::objects;

GraspGoal::GraspGoal(const grasp_helper::GraspPose &grasp_pose_goal, const std::string &description)
: ObjectGoal(grasp_pose_goal, description) {
  goal_.goal_type = "grasp_pose";
}

GraspGoal::GraspGoal(const grasp_helper::Object &object_goal, const std::string &description)
: ObjectGoal(object_goal, description){
  goal_.goal_type = "grasp_pose";
}

jaco_manipulation::PlanAndMoveArmGoal GraspGoal::getGoal() const {
  return ObjectGoal::getGoal();
}

const std::string &GraspGoal::getDescription() const {
  return ObjectGoal::getDescription();
}
