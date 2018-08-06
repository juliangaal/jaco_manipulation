# Jaco Manipulation

Provides ROS Actionlib Server and Client interface to Jaco Robotic Arm.

## Server: jaco\_manipulation_server
### Config
Messages defined in [action/PlanAndMoveArm.action](action/PlanAndMoveArm.action)

### Input
* `geometry_msgs/PoseStamped` or 
* `sensor_msgs/JointState`

## Client: jaco\_manipulation_client
Simple but convenient wrapper around ROS action server SimpleActionClient, tuned for Jaco Robot Arm

### Possible MoveIt Goal States
 * [JointGoal](include/jaco_manipulation/client/goals/joint_goal.h) : Joint state goal state
 * [PoseGoal](include/jaco_manipulation/client/goals/pose_goal.h) : Pose goal state
 * [MoveItGoal](include/jaco_manipulation/client/goals/move_it_goal.h) : Goal state from MoveIt config file ([jaco.srdf](https://github.com/ksatyaki/wpi_jaco/blob/develop/jaco_moveit_config/config/jaco.srdf)) 
 * All goals are defined [here](include/jaco_manipulation/client/goals)
 
### Usage
Start with

```cpp
#include <jaco_manipulation/client/jaco_manipulation_client.h>

int main(int argn, char *args[]) {

  ros::init(argn, args, "pam_client");

  JacoManipulationClient jmc;
```
MoveIt! Config file goal "home"

```cpp
  jmc.moveTo("home");
```

Joint state goal

```cpp
  JointState joint_state;
  joint_state.position.push_back(-2.6435937802859897);
  joint_state.position.push_back(2.478897506888874);
  joint_state.position.push_back(1.680057969995632);
  joint_state.position.push_back(-2.0813597278055846);
  joint_state.position.push_back(1.451960752633381);
  joint_state.position.push_back(1.0931317536782839);
  jmc.moveTo(joint_state);

  
  jmc.moveTo("joint_state_2");
```
Pose goal

```cpp
  PoseStamped pose;
  pose.pose.position.x = 0.063846;
  pose.pose.position.y = -0.193645;
  pose.pose.position.z = 0.509365;
  pose.pose.orientation.x = 0.369761;
  pose.pose.orientation.y = -0.555344;
  pose.pose.orientation.z = -0.661933;
  pose.pose.orientation.w = 0.341635;
  jmc.moveTo(pose);
```
Complete example can be found [here](test/jaco_manipulation_client_test.cpp).


## TODO
###put_down\_points_generator
===> This generates points on a table where
an object currently being held can be safely put down. Relies on
cluster_extraction to extract the centroids of the clusters and the
table. 

###grasp\_pose_generator
===> This creates 4 semi-hardcoded grasp poses
using the position of the object to be grasped. It's a ros server
interface. 
