//
// Created by julian on 04.08.18.
//

#ifndef PROJECT_POSEGOAL_H
#define PROJECT_POSEGOAL_H

#include "move_it_goal.h"

class PoseGoal : public MoveItGoal {
    PoseGoal() = delete;
    PoseGoal(const string &name);
    PoseGoal(const string &name, const geometry_msgs::PoseStamped &pose);
    virtual ~PoseGoal() = default;

    virtual jaco_manipulation::PlanAndMoveArmGoal getGoal() const final;

    void setGoal(const geometry_msgs::PoseStamped &pose);
};


#endif //PROJECT_POSEGOAL_H
