//
// Created by julian on 04.08.18.
//

#ifndef PROJECT_MOVEITGOAL_H
#define PROJECT_MOVEITGOAL_H

#include <actionlib/client/simple_action_client.h>
#include <jaco_manipulation/PlanAndMoveArmAction.h>
#include <jaco_manipulation/server/jaco_manipulation.h>
#include <string>

using std::string;

using PamClient = actionlib::SimpleActionClient<jaco_manipulation::PlanAndMoveArmAction>;

class MoveItGoal {
public:
    MoveItGoal();
    MoveItGoal(const string &name);
    virtual ~MoveItGoal() = default;

    virtual jaco_manipulation::PlanAndMoveArmGoal getGoal() const;

protected:
    jaco_manipulation::PlanAndMoveArmGoal goal;
    const string planning_frame;
};


#endif //PROJECT_MOVEITGOAL_H
