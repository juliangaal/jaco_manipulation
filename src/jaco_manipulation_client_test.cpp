#include <actionlib/client/simple_action_client.h>
#include <jaco_manipulation/PlanAndMoveArmAction.h>
#include <vector>
#include <string>

#define ROS_SUCCESS(x) ROS_INFO_STREAM("\033[32m" << x << "\033[00m")

using std::vector;
using std::string;
using PamClient = actionlib::SimpleActionClient<jaco_manipulation::PlanAndMoveArmAction>;

struct Goal {
  jaco_manipulation::PlanAndMoveArmGoal goal;
  string description;
};

struct Move {
  Goal start;
  Goal end;
  string description;
};

class Executer {
 public:
  Executer() = delete;

  Executer(PamClient &client) : client_(client) {
    moves.reserve(2);
    client_.waitForServer();
    ROS_INFO("Calling jaco_manipulation...");
    execute();
  }

  ~Executer() = default;

  void execute() {
    for (const auto &move: moves) sendGoal(move);
  }

  void addMove(const Move &move) {
    moves.emplace_back(move);
  }

 private:
  void sendGoal(const Move &move) {
    ROS_INFO("----");
    ROS_INFO_STREAM("Attempt: " << move.description);

    if (move.start.goal.goal_type.empty() && moveArm(move.end)) // start goal emoty: start at current state
      ROS_SUCCESS("Success: " + move.description);
    else if (moveArm(move.start) && moveArm(move.end)) // start and goal state occupied
      ROS_SUCCESS("Success: " + move.description);
    else // if one of start or goal move fails
      ROS_ERROR_STREAM("Fail   : " << move.description);
  }

  bool moveArm(const Goal &goal) {
    if (goal.goal.goal_type.empty())
      return false;

    client_.sendGoal(goal.goal);
    client_.waitForResult();

    if (client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_SUCCESS("Move to " + goal.description + " succeeded.");
    else
      ROS_ERROR_STREAM("Move to " << goal.description << " failed.");
  }

  vector<Move> moves;
  PamClient &client_;
};

int main(int argn, char *args[]) {

  ros::init(argn, args, "pam_client");

  PamClient pam_client("plan_and_move_arm", true);
  Executer exe(pam_client);

  {
    Move move;
    move.description = "Joint target: Initial pos -> home joint state";

    jaco_manipulation::PlanAndMoveArmGoal end_goal;
    end_goal.goal_type = "joint_state_1";
    move.end.goal = end_goal;
    move.end.description = "home joint state";

    exe.addMove(move);
  }

  {
    Move move;
    move.description = "Joint target: home joint state -> joint state 1";

    jaco_manipulation::PlanAndMoveArmGoal start_goal;
    start_goal.goal_type = "home";
    move.start.goal = start_goal;
    move.start.description = "home joint state";

    jaco_manipulation::PlanAndMoveArmGoal end_goal;
    end_goal.goal_type = "joint_state_1";
    move.end.goal = end_goal;
    move.end.description = "joint state 1";

    //exe.addMove(move);
  }

  exe.execute();

  return 0;
}

