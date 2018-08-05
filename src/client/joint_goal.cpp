//
// Created by julian on 04.08.18.
//

#include "../../include/jaco_manipulation/client/joint_goal.h"

JointGoal::JointGoal(const string &name) {
    goal.goal_type = name;
    goal.joint_goal.header.frame_id = planning_frame;
    goal.joint_goal.position.reserve(6);
}

JointGoal::JointGoal(const string &name, const vector<double> &joint_goal) {
    goal.goal_type = name;
    goal.joint_goal.header.frame_id = planning_frame;
    goal.joint_goal.position.reserve(6);
    goal.joint_goal.position = joint_goal;
}

jaco_manipulation::PlanAndMoveArmGoal JointGoal::getGoal() const {
    return goal;
}

void JointGoal::setGoal(const vector<double> &joint_goal) {
    goal.joint_goal.position = joint_goal;
}
