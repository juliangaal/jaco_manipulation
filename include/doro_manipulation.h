/*
 * doro_manipulation.h
 *
 *  Created on: Mar 24, 2014
 *      Author: ace
 */

#ifndef DORO_MANIPULATION_H_
#define DORO_MANIPULATION_H_

#include <pcl_msgs/ModelCoefficients.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include "jaco_msgs/FingerPosition.h"
#include "doro_msgs/GraspPoses.h"

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

#define DIST2D(x,y) sqrt( (x*x) + (y*y) )

/**
 * Convenience class to talk to Moveit-ROS interface.
 */
class DoroManipulation
{

protected:

	/**
	 * The move_group variable.
	 */
	move_group_interface::MoveGroup group;

	/**
	 * The plan variable.
	 */
	moveit::planning_interface::MoveGroup::Plan doro_plan;

	/**
	 * The planning scene interface.
	 * This we use to add obstacles. These obstacles are the planes.
	 */
	moveit::planning_interface::PlanningSceneInterface ps_interface;


	/**
	 * A Planning scene monitor to remove the target object from the planning scene.
	 */
//	planning_scene_monitor::PlanningSceneMonitor ps_monitor;

	/**
	 * Subscriber for the Grasp poses generated by the jaco_hand_pose_generator.
	 */
	ros::Subscriber grasp_pose_sub;

	/**
	 * A publisher to control jaco's hand grip.
	 */
	ros::Publisher finger_pub;

	/**
	 * A Publisher to adjust ptu.
	 */
	ros::Publisher pub_ptu;

	/**
	 * A ROS Subscriber for the table's position.
	 */
	ros::Subscriber table_position_sub;

	/**
	 * A ROS Subscriber for the model coefficients of the table.
	 */
	ros::Subscriber table_coeffs_sub;

	/**
	 * A variable to hold the grasp poses generated by jaco_hand_pose_generator.
	 * Constantly updated by the callback for the grasp poses.
	 */
	doro_msgs::GraspPoses pose_list;

	/**
	 * A variable to hold the table's position.
	 * Constanly updated by callback.
	 */
	geometry_msgs::PointStamped table_position;

	/**
	 * A variable to hold the table's model coefficients.
	 */
	pcl_msgs::ModelCoefficients table_coeffs;

	/**
	 * Convenience variable to get the current pose of jaco.
	 * This is not updated by a callback.
	 */
	geometry_msgs::PoseStamped current_pose;

	tf::TransformListener *tf_listener;
	ros::Publisher velocity_pub;

	/**
	 * The callback for the grasp poses generated by jaco_hand_pose_generator.
	 */
	void graspPosesCallback(const doro_msgs::GraspPosesConstPtr& _list);

	/**
	 * The callback for the model coefficients of the table.
	 */
	void tableCoeffsCallback(const pcl_msgs::ModelCoefficientsConstPtr& _coeffs);

	/**
	 * The callback for the table's position.
	 * This is used by the adjustDoro() function to adjust the robot so that the table is well within it's reach.
	 */
	void tablePositionCallback(const geometry_msgs::PointStampedConstPtr& _table);

	/**
	 * Function to tilt the ptu.
	 */
	 void tiltPtu(float value);

	 bool aligned, near;

public:


	DoroManipulation();
	virtual ~	DoroManipulation();

	/* Callback that stores the pose of the cylinder (in the /jaco_base_link frame)  in the
	 * variable cylinder_pose */
	//void cylinderPoseCallback(const geometry_msgs::PoseStampedConstPtr& cyl_pose);

	/**
	 * A function to add the table as an obstacle.
	 */
	void addTableAsObstacle();

	/**
	 * A function to add the target as an obstacle.
	 */
	void addTargetAsObstacle();

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
	 * Convenience function to plan the pose specified by target_pose.
	 */
	bool plan(const geometry_msgs::PoseStamped& target_pose);

	/**
	 * Function to return the current value of the pose_list.
	 */
	doro_msgs::GraspPoses getCylinderPose();

	/**
	 * Function to return the current value of the table_coeffs.
	 * table_coeffs are contantly updated by tableCoeffsCallback().
	 */
	pcl_msgs::ModelCoefficients getTableCoeffs();

	/**
	 * Function to return the current position of the table.
	 * This position is constantly updated by callback.
	 */
	geometry_msgs::PointStamped getTablePosition();

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
	 * This method helps adjust doro so that the bottle is within reach.
	 */
	void adjustDoro ();
	
	/**
	 * Reset the values after a run.
	 */
	 void resetValues();

};

#endif /* DORO_MANIPULATION_H_ */
