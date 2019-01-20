# How to run these tests

# Rule #1
**Always keep a finger on the power button on Jaco**. Movements can be fast from time to time, and error can occur at any time, though very rarely. The first things to break are Jacos fingers...

### 1. client\_grasp\_test.cpp
* *Use Case*: provides simple interface to Jaco arm to test predefined poses and bounding boxes 
* *Requirements*: `jaco_manipulation_server` needs to be running: `roslaunch jaco_manipulation jaco_manipulation`. If you want rviz visualization, use parameter `use_rviz:=true/false`
* *Run*: `rosrun jaco_manipulation client_grasp_test`
* *Evaluation*: your eyes

### 2. client\_test.cpp
* *Use Case*: provides simple interface to Jaco arm to test predefined poses, joint states and goals defined in MoveIt config
* *Requirements*: `jaco_manipulation_server` needs to be running: `roslaunch jaco_manipulation jaco_manipulation`. If you want rviz visualization, use parameter `use_rviz:=true/false`
* *Run*: `rosrun jaco_manipulation client_test`
* *Evaluation*: your eyes

# More
These two tests are only the very basic ones for quick testing of changes to this package, all other complex tests that were part of testing fro ReGround happen in a [separate package](https://github.com/juliangaal/jaco_manipulation_test)
