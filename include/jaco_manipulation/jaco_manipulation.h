/*
  Copyright (C) 2015  Chittaranjan Srinivas Swaminathan

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>
  
*/
#ifndef JACO_MANIPULATION_H_
#define JACO_MANIPULATION_H_

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <jaco_manipulation/PlanAndMoveArmAction.h>
#include <jaco_manipulation/grasp_pose_generator.h>

#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/JointState.h"

#define HOME_POSE(home_pose) { \
	home_pose.pose.position.x = 0.167935;\
	home_pose.pose.position.y = -0.008380;\
	home_pose.pose.position.z = 1.081672;\
	home_pose.pose.orientation.x = 0.252635;\
	home_pose.pose.orientation.y = 0.641370;\
	home_pose.pose.orientation.z = 0.689190;\
	home_pose.pose.orientation.w = 0.219360;\
	home_pose.header.frame_id = "/base_link";\
}

#define HANDOVER_POSE(h_pose) { \
	h_pose.pose.position.x = 0.552;\
	h_pose.pose.position.y = 0.008;\
	h_pose.pose.position.z = 1.000;\
	h_pose.pose.orientation.x = 0.252635;\
	h_pose.pose.orientation.y = 0.641370;\
	h_pose.pose.orientation.z = 0.689190;\
	h_pose.pose.orientation.w = 0.219360;\
	h_pose.header.frame_id = "/base_link";\
}

namespace jaco_manipulation
{
/**
 * Convenience class to talk to Moveit-ROS interface.
 */
class JacoManipulation
{

protected:

	/**
	 * A common NodeHandle.
	 */
	ros::NodeHandle nh_;

	/**
	 * Action server that is used for manipulation.
	 */
	actionlib::SimpleActionServer <jaco_manipulation::PlanAndMoveArmAction> pam_server_;

	/**
	 * The plan variable.
	 */
	moveit::planning_interface::MoveGroup::Plan plan_;

	/**
	 * The planning scene interface.
	 * This we use to add obstacles. These obstacles are the planes.
	 */
	moveit::planning_interface::PlanningSceneInterface ps_interface_;

	/**
	 * A Planning scene monitor to remove the target object from the planning scene.
	 */
//	planning_scene_monitor::PlanningSceneMonitor ps_monitor;

	/**
	 * A publisher to control jaco's hand grip.
	 */
	ros::Publisher finger_pub_;

	/**
	 * A Publisher to adjust ptu.
	 */
	ros::Publisher pub_ptu_;

	/**
	 * Convenience variable to get the current pose of jaco.
	 * This is not updated by a callback.
	 */
	geometry_msgs::PoseStamped current_pose_;

	boost::shared_ptr<tf::TransformListener> tf_listener_;

	/**
	 * Function to tilt the ptu.
	 */
	 void tiltPtu(float value);

	 /**
	  * Callback for the action server.
	  */
	 void processGoal(const jaco_manipulation::PlanAndMoveArmGoalConstPtr& _goal);

public:

	 /**
	  * The move_group variable.
	  */
	move_group_interface::MoveGroup group_;

	JacoManipulation();
	virtual ~JacoManipulation();

	/**
	 * A function to add the table as an obstacle.
	 */
	void addTableAsObstacle(geometry_msgs::PoseStamped table_pose);

	/**
	 * A function to add the target as an obstacle.
	 */
	void addTargetAsObstacle(geometry_msgs::PoseStamped box_pose);

	/**
	 * Remove the Table after task is complete.
	 */
	void removeTable();

	/**
	 * Remove the Target after release.
	 */
	void removeTarget();

	// Sucky
	void attachTarget();

	// Sucky 2
	void detachTarget();

	/**
	 * Convenience function to plan and execute the pose specified by target_pose.
	 */
	bool planAndMove(const geometry_msgs::PoseStamped& target_pose);
	
	/**
	 * Convenience function to plan and execute the pose specified by string.
	 */
	bool planAndMove(const std::string& target_pose_string);

	/**
	 * Convenience function to plan the pose specified by target_pose.
	 */
	bool plan(const geometry_msgs::PoseStamped& target_pose);

	/**
	 * A function to close doro's hand.
	 */
	void closeHand(float value = 1.0);

	/**
	 * A function to open doro's hand.
	 */
	void openHand();

	/**
	 * This function returns true if the arm has reached the target pose.
	 */
	bool hasReachedPose (const geometry_msgs::PoseStamped& target_pose);

	/**
	 * Reset the values after a run.
	 */
	void resetValues();

};
}
#endif /* JACO_MANIPULATION_H_ */
