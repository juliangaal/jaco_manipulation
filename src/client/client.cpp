//
// Created by julian on 05.08.18.
//

#include "jaco_manipulation/client/client.h"

namespace Moveit {

    Client::Client() : client_("plan_and_move_arm", true) {
        client_.waitForServer();
    }

    Client::~Client() {
        client_.stopTrackingGoal();
        client_.cancelAllGoals();
    }

    template<typename T>
    void Client::execute(const T &goal) {
        client_.sendGoal(goal.getGoal());
        client_.waitForResult();

        if (client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_SUCCESS("Status : Move to " + goal.goal_type + " succeeded.");
        } else {
            ROS_ERROR_STREAM("Status : Move to " << goal.goal_type << " failed.");
        }
    }
}