#include <doro_manipulation/GenerateGraspPoses.h>
#include <ros/ros.h>

int main(int argn, char* args[])
{
	ros::init(argn, args, "gpg_test_client");
	ros::NodeHandle nh;

	doro_manipulation::GenerateGraspPoses message;

	message.request.object_location.point.x = 0.750812;
	message.request.object_location.point.y = -0.0176425;
	message.request.object_location.point.z = 0.897799;

	message.request.object_location.header.frame_id = "base_link";
	message.request.object_location.header.stamp = ros::Time::now();


	ros::ServiceClient cliend = nh.serviceClient <doro_manipulation::GenerateGraspPoses> ("generate_grasp_poses", false);

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
