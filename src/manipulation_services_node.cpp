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
#include <ros/ros.h>
#include <jaco_manipulation/grasp_pose_generator.h>
#include <jaco_manipulation/server/jaco_manipulation_server.h>

using namespace jaco_manipulation;

int main(int argn, char *args[]) {
  ros::init(argn, args, "manipulation_server");
  ros::MultiThreadedSpinner m_t_spinner(4);

  ROS_INFO("Starting server for manipulation.");
  server::JacoManipulation dmt;

  ROS_INFO("Starting server for grasp pose generation.");
  jaco_manipulation::GraspPoseGenerator GPG_server;

  ROS_INFO("All servers are up.");

  m_t_spinner.spin();
}


