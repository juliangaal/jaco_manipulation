#include "doro_moveit.h"
#include "jaco_hand_pose_generator.h"

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <pthread.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

JacoHandPoseGenerator *JHPG;

#define DIST2D(x,y) sqrt( (x*x) + (y*y) )

bool move_base_doro (move_base_msgs::MoveBaseGoal move_base_goal)
{
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> sac("move_base", true);

	ROS_INFO("WAITING...");
	sac.waitForServer();
	ROS_INFO("SERVER DETECTED...");

	sac.sendGoal(move_base_goal);

	ROS_INFO("GOAL SENT. WAITING FOR RESULT!");
	sac.waitForResult();

	if(sac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
			ROS_INFO("SUCCESS!");
			return true;
	}
	else
	{
		ROS_INFO("FAILED!");
		return false;
	}


}

void *spin_thread(void* nol)
{
	ros::spin();
	return NULL;
}

void start_cylinder_extraction ()
{
	JHPG = new JacoHandPoseGenerator;
	ROS_INFO("STARTING CYLINDER EXTRACTION!");
	sleep(5);
	ROS_INFO("LISTENED FOR 5 SECONDS!");
}

bool moveit_doro ()
{
	DoroMoveit *D_M_T (new DoroMoveit);

	geometry_msgs::PoseStamped sg_pose = D_M_T->getCylinderPose();

	ROS_INFO("The CYLINDER POSE: (%f,%f,%f) ; (%f,%f,%f,%f)",
			sg_pose.pose.position.x,
			sg_pose.pose.position.y,
			sg_pose.pose.position.z,
			sg_pose.pose.orientation.x,
			sg_pose.pose.orientation.y,
			sg_pose.pose.orientation.z,
			sg_pose.pose.orientation.w);

	while((sg_pose.pose.position.x == sg_pose.pose.position.y) && (sg_pose.pose.position.x == sg_pose.pose.position.z)
	&& (sg_pose.pose.position.x == 0.0) && (sg_pose.header.seq < 5) )
	{
		ROS_INFO("WE ARE WAITING FOR CYLINDER POSE!");
		sg_pose = D_M_T->getCylinderPose();
		sleep(1);
	}

	// **********************************************************************************
	// ADJUST THE ROBOT SO THAT IT IS MORE OR LESS WITHIN THE REACH OF THE
	// **********************************************************************************
	ros::NodeHandle n;
	ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	geometry_msgs::Twist adjust_vel;

	geometry_msgs::PointStamped cylinder_in_openni_frame, cylinder_in_base_link_frame;
	cylinder_in_openni_frame.point = sg_pose.pose.position;
	cylinder_in_openni_frame.header = sg_pose.header;

	tf::TransformListener *tf = JHPG->tf_;

	try
	{
		tf->transformPoint("base_link", cylinder_in_openni_frame, cylinder_in_base_link_frame);
	}
	catch(tf::TransformException& ex)
	{
		ROS_INFO("Can't do a transform! Something went wrong.");
		exit(0);
	}

	float dist_to_cylinder = DIST2D(cylinder_in_base_link_frame.point.x,cylinder_in_base_link_frame.point.y);

	if( dist_to_cylinder > 0.7)
	{
		ROS_INFO("The distance is now: %f", dist_to_cylinder);
		ROS_INFO("ADJUSTING!");

		adjust_vel.linear.x = dist_to_cylinder-0.6;
		adjust_vel.angular.z = atan2(cylinder_in_base_link_frame.point.y,cylinder_in_base_link_frame.point.x);

		sleep(1);
		velocity_pub.publish(adjust_vel);
		sleep(1);
	}

	sleep(4);

	sg_pose = D_M_T->getCylinderPose();
	// **********************************************************************************


	geometry_msgs::PoseStamped sg_pose_1;
	geometry_msgs::PoseStamped home_pose;

	HOME_POSE(home_pose);

	geometry_msgs::Point diff;

	diff.x = -0.00;
	diff.y = -0.00;
	diff.z = +0.06;

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
	sg_pose_1.pose.position.z = sg_pose.pose.position.z + diff.z;

	sg_pose.pose.position.x = sg_pose.pose.position.x + 0.030;
	sg_pose.pose.position.y = sg_pose.pose.position.y + 0.030;
	sg_pose.pose.position.z = sg_pose.pose.position.z + 0.020;

	// Move near
	D_M_T->openHand();
	D_M_T->planAndMove(sg_pose_1);

	// Move to the spot
	D_M_T->openHand();
	D_M_T->planAndMove(sg_pose);

	// Grab it
	D_M_T->closeHand();

	// Take me home
	D_M_T->planAndMove(home_pose);
	D_M_T->openHand();

	delete D_M_T;

	return true;
}

int main(int argn, char* args[])
{
	ros::init(argn, args, "lets_move_it_doro");
	ros::NodeHandle n;

	pthread_t id;

	pthread_create(&id, NULL, spin_thread, NULL);

	// Create a goal message and dispatch it
	move_base_msgs::MoveBaseGoal move_base_goal;

	move_base_goal.target_pose.pose.position.x = 4.900;
	move_base_goal.target_pose.pose.position.y = -1.257;
	move_base_goal.target_pose.pose.orientation.z = -0.687;
	move_base_goal.target_pose.pose.orientation.w = 0.727;

	move_base_goal.target_pose.header.stamp = ros::Time::now();
	move_base_goal.target_pose.header.frame_id = "map";

	//move_base_doro (move_base_goal);

	start_cylinder_extraction ();

	moveit_doro ();

	delete JHPG;

	pthread_join(id, NULL);

}
