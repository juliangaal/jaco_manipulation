#include "doro_moveit.h"
#include "jaco_hand_pose_generator.h"

void *spin_thread(void* nol)
{
	ros::spin();
	return NULL;
}

int main(int argn, char* args[])
{
	ros::init(argn, args, "lets_move_it_doro");

	JacoHandPoseGenerator JHPG;
	pthread_t id;
	pthread_create(&id, NULL, spin_thread, NULL);
	sleep(5);


	DoroMoveit D_M_T;

	geometry_msgs::PoseStamped sg_pose = D_M_T.getCylinderPose();

	ROS_INFO("The CYLINDER POSE: (%f,%f,%f) ; (%f,%f,%f,%f)",
			sg_pose.pose.position.x,
			sg_pose.pose.position.y,
			sg_pose.pose.position.z,
			sg_pose.pose.orientation.x,
			sg_pose.pose.orientation.y,
			sg_pose.pose.orientation.z,
			sg_pose.pose.orientation.w);

	while((sg_pose.pose.position.x == sg_pose.pose.position.y) && (sg_pose.pose.position.x == sg_pose.pose.position.z)
	&& (sg_pose.pose.position.x == 0.0))
	{
		ROS_INFO("WE ARE WAITING FOR CYLINDER POSE!");
		sg_pose = D_M_T.getCylinderPose();
		sleep(1);
	}

	geometry_msgs::PoseStamped sg_pose_1;
	geometry_msgs::PoseStamped home_pose;

	HOME_POSE(home_pose);

	geometry_msgs::Point diff;

	diff.x = -0.1;
	diff.y = -0.08;
	diff.z = 0;

	/* 	sg_pose.header.frame_id = "/base_link";
	sg_pose.header.stamp = ros::Time::now();

	sg_pose.pose.position.x = 0.623721;
	sg_pose.pose.position.y = -0.006068;
	sg_pose.pose.position.z = 1.077136;

	sg_pose.pose.orientation.x = 0.258182;
	sg_pose.pose.orientation.y = 0.640500;
	sg_pose.pose.orientation.z = 0.689190;
	sg_pose.pose.orientation.w = 0.219360; *********************/

	sg_pose_1.pose.orientation = sg_pose.pose.orientation;
	sg_pose_1.header.frame_id = sg_pose.header.frame_id;

	sg_pose_1.pose.position.x = sg_pose.pose.position.x + diff.x;
	sg_pose_1.pose.position.y = sg_pose.pose.position.y + diff.y;
	sg_pose_1.pose.position.z = sg_pose.pose.position.z;

	sg_pose.pose.position.x = sg_pose.pose.position.x + 0.030;
	sg_pose.pose.position.y = sg_pose.pose.position.y + 0.030;

	// Move near
	D_M_T.openHand();
	D_M_T.planAndMove(sg_pose_1);

	// Move to the spot
	D_M_T.openHand();
	D_M_T.planAndMove(sg_pose);

	// Grab it
	D_M_T.closeHand();

	// Take me home
	D_M_T.planAndMove(home_pose);
	D_M_T.openHand();


	pthread_join(id, NULL);

}
