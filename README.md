# Jaco Manipulation

Provides ROS Actionlib Server and Client interface to Jaco Robotic Arm as part of the [ReGround Projekt]().

<p align="center"><a href="https://imgur.com/Jrbe7nQ"><img src="https://i.imgur.com/Jrbe7nQ.jpg" title="source: imgur.com" /></a></p>

*Fun Fact: the arm (gen 1) made it to the [NYTimes](https://www.nytimes.com/2018/10/21/business/what-comes-after-the-roomba.html), although only the title picture...*

Depends on [wpi_jaco](https://github.com/ksatyaki/wpi_jaco), a custom driver for the robotic arm and [rail_manipulation_msgs](https://github.com/GT-RAIL/rail_manipulation_msgs). Just drop the repos in your catkin workspace, or use the provided `.rosinstall` file to start a fresh workspace with [wstool](http://wiki.ros.org/wstool).

Depends on installation of [MoveIt!](https://moveit.ros.org/) and it's [visual tools](https://github.com/ros-planning/moveit_visual_tools). They can be installed with [this](https://github.com/juliangaal/reground_workspace/blob/reground/install/.install_apts.sh) script or with apt.

Test the installation 
```
cd <workspace_root>
catkin_make -j8
source devel/setup.<bash/zsh>
roslaunch manipulation_launch manipulation.launch use_rviz:=true
```

An RViz window should open with Moveit! ready to be used

*Note*: If you have any problems with dependencies, try running `rosdep install --from-paths src -i -y`

## Testing
All tests with the ReGround anchoring system in place have been moved to a [separate ROS package](https://github.com/juliangaal/jaco_manipulation_test).

## Server: jaco\_manipulation_server
### Config
Define these ROS parameters in the [launch file](launch/jaco_manipulation.launch)
* `planner_id type`: e.g. RRTConnectkConfigDefault, RRTstarkConfigDefault, etc. **Default**: RRTConnectkConfigDefault
* `planning_time`: time in seconds **Default**: 1.5
* `planning_attempts`: number of planning attempts **Default**: 10
* `allow_replanning`: true/false **Default**: true
* `allow_looking`: true/false **Default**: true
* `publish_debug`: true/false. **Default**: true. If true, [this message](msg/JacoDebug.msg) is published on topic `/jaco_manipulation/debug`

Messages defined in [action/PlanAndMoveArm.action](action/PlanAndMoveArm.action)

### Input
* `geometry_msgs/PoseStamped` or 
* `sensor_msgs/JointState` and when used with vision system:
* `jaco_manipulation/BoundingBox bounding_box` defined [here](include/jaco_manipulation/goals/goal_input.h)

For pose and joint state it is assumed that they are valid poses. If they are not, MoveIt! can't find a plan. If you use the bounding box, certain parameters are adjusted, like height, so the object can never be e.g. under the table.

**Note**: the bounding box should be defined with a centroid and dimensions. If your bounding box is defined differently, e.g. by the the position of the lower left corner and dimensions, this code will still assume this to be a centroid. Adjust your definition of a bounding box accordingly.

## Client: jaco\_manipulation_client
Simple but convenient wrapper around ROS action server SimpleActionClient, tuned for Jaco Robot Arm. Defined [here](include/jaco_manipulation/client/jaco_manipulation_client.h)

### Possible MoveIt Goal States
 * [JointGoal](include/jaco_manipulation/goals/joint_goal.h) : Joint state goal state
 * [PoseGoal](include/jaco_manipulation/goals/pose_goal.h) : Pose goal state
 * [MoveItGoal](include/jaco_manipulation/goals/move_it_goal.h) : Goal state from MoveIt config file ([jaco.srdf](https://github.com/ksatyaki/wpi_jaco/blob/develop/jaco_moveit_config/config/jaco.srdf)) 
 * [DropGoal](include/jaco_manipulation/goals/drop_goal.h) : Bounding Box to be dropped
 * [GraspGoal](include/jaco_manipulation/goals/grasp_goal.h) : Bounding Box to be grasped
 * All goals are defined [here](include/jaco_manipulation/goals) 

### Usage
> *WARNING* You are advised to use these defined literals in order to avoid any metric confusion in all cases except JointState and Pose Quaternion definition. A zero too many or little happens very quick and can lead to unreachable poses. Additionally, it clearly states intent.
```cpp
double a = 1.0_m;  // 1 meter
double b = 1.0_cm; // 1 centimeter, or 0.01
double c = 1.0_mm; // 1 mm, or 0.001
```
They are defined [here](include/jaco_manipulation/units.h). This may save you [$125mio](http://articles.latimes.com/1999/oct/01/news/mn-17288)

Start with

```cpp
#include <jaco_manipulation/client/jaco_manipulation_client.h>

using namespace jaco_manipulation::client;

int main(int argn, char *args[]) {

  ros::init(argn, args, "pam_client");

  JacoManipulationClient jmc;
```
MoveIt! Config file goal "home". Config file goals are defined [in the driver](https://github.com/ksatyaki/wpi_jaco/blob/18aa79e541a35c8cb288c2d2a9842870894a632e/jaco_moveit_config/config/jaco.srdf#L32)

```go
jmc.moveTo("home");
```

Pose goal

```go
geometry_msgs::PoseStamped pose;
pose.pose.position.x = 0.5_m;
pose.pose.position.y = 0.3_m;
pose.pose.position.z = 0.0_cm;
pose.pose.orientation.x = 0.369761;
pose.pose.orientation.y = -0.555344;
pose.pose.orientation.z = -0.661933;
pose.pose.orientation.w = 0.341635;
jmc.moveTo(pose);
```

Joint state goal

```go
sensor_msgs::JointState joint_state;
joint_state.position.push_back(-2.6435937802859897);
joint_state.position.push_back(2.478897506888874);
joint_state.position.push_back(1.680057969995632);
joint_state.position.push_back(-2.0813597278055846);
joint_state.position.push_back(1.451960752633381);
joint_state.position.push_back(1.0931317536782839);
jmc.moveTo(joint_state);
```

Grasp goal
```go
jaco_manipulation::BoundingBox b;
b.header.frame_id = "base_link";
b.description = "bottle";
b.point.x = 0.65_m;
b.point.y = 30._cm;
b.point.z = 10._cm;
b.dimensions.x = 6._cm;
b.dimensions.y = 6._cm;
b.dimensions.z = 21._cm;
jmc.graspAt(b);
```

Drop goal
```go
jaco_manipulation::BoundingBox b;
b.header.frame_id = "base_link";
b.description = "bottle";
b.point.x = 55._cm;
b.point.y = 0.0_cm;
b.point.z = 10._cm;
b.dimensions.x = 6._cm;
b.dimensions.y = 6._cm;
b.dimensions.z = 21._cm;
jmc.dropAt(b);
```

Examples can be found [here](test/).
