/*
 * manipulation_services_node.cpp
 *
 *  Created on: Oct 3, 2014
 *      Author: ace
 */

#include <ros/ros.h>
#include <doro_manipulation/grasp_pose_generator.h>
#include <doro_manipulation/doro_manipulation.h>
#include <doro_manipulation/put_down_points_generator.h>

int main(int argn, char* args[])
{
	ros::init(argn, args, "manipulation_server");
	ros::MultiThreadedSpinner m_t_spinner(4);

	ROS_INFO("Starting server for manipulation.");
	doro_manipulation::DoroManipulation dmt;
	ROS_INFO("Starting server for grasp pose generation.");
	doro_manipulation::GraspPoseGenerator GPG_server;
	ROS_INFO("Starting server for put down points generation.");
	doro_manipulation::PutDownPointsGenerator PDPG_server;

	ROS_INFO("All servers are up.");

	m_t_spinner.spin();
}


