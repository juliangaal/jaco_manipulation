#include "doro_moveit.h"
#include "grasp_pose_generator.h"

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <pthread.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <vector>

#include "doro_msgs/GraspPoses.h"

#define DIST2D(x,y) sqrt( (x*x) + (y*y) )

GraspPoseGenerator *JHPG;

void *spin_thread(void* nol)
{
	ros::spin();
	return NULL;
}

void start_cylinder_extraction ()
{
	JHPG = new GraspPoseGenerator;
	ROS_INFO("STARTING CYLINDER EXTRACTION!");
	sleep(6);
	ROS_INFO("LISTENED FOR 5 SECONDS!");
}

bool moveit_doro ()
{
	DoroMoveit *D_M_T (new DoroMoveit);
	sleep(4);

	//D_M_T->adjustDoro();

	doro_msgs::GraspPoses poses = D_M_T->getCylinderPose();

	while((poses.pregrasp_poses[0].pose.position.x == poses.pregrasp_poses[0].pose.position.y) && (poses.pregrasp_poses[0].pose.position.x == poses.pregrasp_poses[0].pose.position.z)
	&& (poses.pregrasp_poses[0].pose.position.x == 0.0) && (poses.pregrasp_poses[0].header.seq < 5) )
	{
		ROS_INFO("WE ARE WAITING FOR CYLINDER POSE!");
		poses = D_M_T->getCylinderPose();
		sleep(1);
	}

	//D_M_T->adjustDoro();
	//poses = D_M_T->getCylinderPose();


	geometry_msgs::PoseStamped home_pose;

	HOME_POSE(home_pose);

	std::vector<geometry_msgs::PoseStamped>::iterator gp, pgp;

	gp = poses.grasp_poses.begin();
	pgp = poses.pregrasp_poses.begin();

	bool moved = false;

	while(gp!= poses.grasp_poses.end() && pgp != poses.pregrasp_poses.end())
	{

		// Move near
		D_M_T->openHand();
		moved = D_M_T->planAndMove(*pgp);
		if(moved)
		{
			ROS_INFO("Pre-grasp success!");
			ROS_INFO("Moving to the grasp pose...");
			// Move to the spot
			D_M_T->openHand();
			moved = D_M_T->planAndMove(*gp);

			D_M_T->closeHand();
			moved = D_M_T->planAndMove(home_pose);

			if(moved)
				break;


		}

		pgp++;
		gp++;

	}
	delete D_M_T;
	return true;
}

int main(int argn, char* args[])
{
	ros::init(argn, args, "lets_move_it_doro");
	ros::NodeHandle n;

	pthread_t id;

	pthread_create(&id, NULL, spin_thread, NULL);
	start_cylinder_extraction ();

	moveit_doro ();

	delete JHPG;

}
