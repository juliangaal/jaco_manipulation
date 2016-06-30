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

#include <jaco_manipulation/GenerateGraspPoses.h>
#include <ros/ros.h>

int main(int argn, char* args[])
{
  ros::init(argn, args, "gpg_test_client");
  ros::NodeHandle nh;

  jaco_manipulation::GenerateGraspPoses message;

  message.request.object_location.point.x = 0.0;
  message.request.object_location.point.y = -0.20;
  message.request.object_location.point.z = 0.50;
  message.request.object_location.header.frame_id = "base_link";
  message.request.object_location.header.stamp = ros::Time::now();


  ros::ServiceClient cliend = nh.serviceClient <jaco_manipulation::GenerateGraspPoses> ("generate_grasp_poses", false);

  if(cliend.call(message))
  {
    std::cout<<message.response;
  }
  else
  {
    ROS_ERROR("Fails");
    std::cout<<message.response;
  }
}
