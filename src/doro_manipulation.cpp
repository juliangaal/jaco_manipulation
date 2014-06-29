/*
 * doro_manipulation.cpp
 *
 *  Created on: Mar 24, 2014
 *      Author: Chittaranjan Srinivas Swaminathan
 */

#include "doro_manipulation.h"

DoroManipulation::DoroManipulation() : group("arm")
{
	// TODO Auto-generated constructor stub
	ROS_INFO("Initializing Doro Manipulation!");

	ros::NodeHandle nh;

	grasp_pose_sub = nh.subscribe("/grasp_poses", 1, &DoroManipulation::graspPosesCallback, this);

	table_position_sub = nh.subscribe("/table_position", 1, &DoroManipulation::tablePositionCallback, this);

	table_coeffs_sub = nh.subscribe("/table_coeffs", 1, &DoroManipulation::tableCoeffsCallback, this);

	velocity_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	finger_pub = nh.advertise<jaco_msgs::FingerPosition>("/cmd_abs_finger", 1);
	pub_ptu = nh.advertise<sensor_msgs::JointState>("/ptu/cmd", 1);

	pose_list.grasp_poses.resize(4);
	pose_list.pregrasp_poses.resize(4);

	table_coeffs.values.resize(4);
	tf_listener = new tf::TransformListener(ros::Duration(10));
	
	group.setPlanningTime(10);

	aligned = false;
	near = false;
}

void DoroManipulation::resetValues()
{
	pose_list.grasp_poses.clear();
	pose_list.pregrasp_poses.clear();
	table_coeffs.values.clear();
	
	pose_list.grasp_poses.resize(4);
	pose_list.pregrasp_poses.resize(4);
	table_coeffs.values.resize(4);
}

void DoroManipulation::tiltPtu(float value)
{
	ROS_INFO("PTU IS BEING TILTED!");
	sensor_msgs::JointState ptu_msg;
	ptu_msg.header.stamp = ros::Time::now() + ros::Duration(0.01);

	ptu_msg.name.resize(2);
	ptu_msg.position.resize(2);
	ptu_msg.velocity.resize(2);
	ptu_msg.effort.resize(2);

	ptu_msg.name[0] = "ptu_pan_joint";
	ptu_msg.name[1] = "ptu_tilt_joint";

	ptu_msg.position[0]=0.0;
	ptu_msg.position[1]=value;

	ptu_msg.velocity[0]=0.5;
	ptu_msg.velocity[1]=0.5;

	sleep(1);
	pub_ptu.publish(ptu_msg);
	sleep(1);

	ROS_INFO("TILTED PTU");
}

/**
 *  Callback that stores the pose of the cylinder (in the /jaco_base_link frame)  in the
 *  variable cylinder_pose
 */
void DoroManipulation::graspPosesCallback(const doro_msgs::GraspPosesConstPtr& _list)
{
	//ROS_INFO("CALLBACK CYLINDER!");
	pose_list = *_list;
	//ROS_INFO("CALLBACK CYLINDER DONE!");
}

/**
 *  Function to return the current value of the cylinder pose
 */
doro_msgs::GraspPoses DoroManipulation::getCylinderPose()
{
	return pose_list;
}

DoroManipulation::~DoroManipulation()
{
	if(tf_listener)
		delete tf_listener;

	grasp_pose_sub.shutdown();
	table_position_sub.shutdown();
	table_coeffs_sub.shutdown();
}

void DoroManipulation::tablePositionCallback(const geometry_msgs::PointStampedConstPtr& _table)
{
	table_position = *_table;
}

geometry_msgs::PointStamped DoroManipulation::getTablePosition()
{
	return table_position;
}

void DoroManipulation::tableCoeffsCallback(const pcl_msgs::ModelCoefficientsConstPtr& _table_coeffs)
{
	table_coeffs = *_table_coeffs;
}

pcl_msgs::ModelCoefficients DoroManipulation::getTableCoeffs()
{
	return table_coeffs;
}

void DoroManipulation::adjustDoro()
{
	ros::NodeHandle nh;
	table_position_sub = nh.subscribe("/table_position", 1, &DoroManipulation::tablePositionCallback, this);

	/* THIS CODE WAS INSIDE TABLE POSITION CALLBACK. FIND A WAY!
	 * geometry_msgs::Twist adjust_vel;

	double dist_to_table = DIST2D(_table->point.x,_table->point.y);

	if(dist_to_table > 0.89)
	{
		dist_to_table = DIST2D(_table->point.x ,_table->point.y);

		ROS_INFO("Table is at (%f, %f) from doro. [%d]", _table->point.x, _table->point.y, _table->header.seq);
		ROS_INFO("Table is %fm from doro.", dist_to_table);

		if(fabs(_table->point.y) < 0.1)
			aligned = true;
		else
			aligned = false;

		if(!near)
			adjust_vel.linear.x = 0.1;
		else
			adjust_vel.linear.x = 0.0;

		if(!aligned)
			adjust_vel.angular.z = (_table->point.y > 0? 1 : -1)*0.1;
		else
			adjust_vel.angular.z = 0.0;

		velocity_pub.publish(adjust_vel);
	}

	else
	{
		ROS_INFO("Target seems to be in range. Let's go, then.");
		near = true;
		aligned = true;
	}
	 */

	tiltPtu(-0.5);
	ros::Rate callback_rate(0.5);
	while(!near)
	{
		ros::spinOnce();
		callback_rate.sleep();
	}
	table_position_sub.shutdown();
}

bool DoroManipulation::hasReachedPose (const geometry_msgs::PoseStamped& target_pose)
{
	geometry_msgs::PoseStamped now_pose = group.getCurrentPose();

	if(
			(now_pose.pose.position.x - target_pose.pose.position.x < 0.01) &&
			(now_pose.pose.position.y - target_pose.pose.position.y < 0.01) &&
			(now_pose.pose.position.z - target_pose.pose.position.z < 0.01) &&
			(now_pose.pose.orientation.x - target_pose.pose.orientation.x < 0.01) &&
			(now_pose.pose.orientation.y - target_pose.pose.orientation.y < 0.01) &&
			(now_pose.pose.orientation.z - target_pose.pose.orientation.z < 0.01) &&
			(now_pose.pose.orientation.w - target_pose.pose.orientation.w < 0.01) )
	{
		ROS_INFO("Pose reached. Motion complete.");
		return true;
	}
	else
	{
		ROS_INFO("Moving...");
		return false;
	}

}

/**
 * Convenience function to plan and execute the pose specified by target_pose
 */
bool DoroManipulation::planAndMove(const geometry_msgs::PoseStamped& target_pose)
{

	group.allowReplanning(true);
	group.allowLooking(true);

	group.setStartStateToCurrentState();
	
	//ROS_INFO("CURRENT STATE SET!");


	group.setPoseReferenceFrame (target_pose.header.frame_id);
	group.setPoseTarget(target_pose);
	//group.setRandomTarget();
	
	ROS_INFO("TARGET POSE SET!");
	

	geometry_msgs::PoseStamped the_pose_now = group.getCurrentPose();

	ROS_INFO("The pose now: (%f,%f,%f) ; (%f,%f,%f,%f)",
			the_pose_now.pose.position.x,
			the_pose_now.pose.position.y,
			the_pose_now.pose.position.z,
			the_pose_now.pose.orientation.x,
			the_pose_now.pose.orientation.y,
			the_pose_now.pose.orientation.z,
			the_pose_now.pose.orientation.w);
		ROS_INFO("NOW FRAME:= %s",the_pose_now.header.frame_id.c_str());

	ROS_INFO("The target pose: (%f,%f,%f) ; (%f,%f,%f,%f)",
			target_pose.pose.position.x,
			target_pose.pose.position.y,
			target_pose.pose.position.z,
			target_pose.pose.orientation.x,
			target_pose.pose.orientation.y,
			target_pose.pose.orientation.z,
			target_pose.pose.orientation.w);
	ROS_INFO("FRAME FOR PLANNING:= %s",target_pose.header.frame_id.c_str());



	ROS_INFO("FRAME FOR PLANNING:= %s",group.getPoseReferenceFrame().c_str());
	bool success = group.plan(doro_plan);

	if(success)
		ROS_INFO("WOW A PLAN WAS FOUND!!!");
	else
	{
		ROS_INFO("SOMETHING SUCKED AND WE FAILED MISERABLY!!!");
		return false;
	}

	if(group.move())
	{
		ROS_INFO("WE MOVED MAN!!!");
		while(!hasReachedPose(target_pose))
		{
			sleep(1);
		}
		return true;
	}
	else
	{
		ROS_INFO("SUCKS! LIFE SUCKS!!!");
		return false;
	}

}
/**
 * Convenience function to plan the pose specified by target_pose
 */
bool DoroManipulation::plan(const geometry_msgs::PoseStamped& target_pose)
{

	group.setStartStateToCurrentState();

	group.setPoseReferenceFrame (target_pose.header.frame_id);
	group.setPoseTarget(target_pose);
	geometry_msgs::PoseStamped the_pose_now = group.getCurrentPose();

	ROS_INFO("The pose now: (%f,%f,%f) ; (%f,%f,%f,%f)",
			the_pose_now.pose.position.x,
			the_pose_now.pose.position.y,
			the_pose_now.pose.position.z,
			the_pose_now.pose.orientation.x,
			the_pose_now.pose.orientation.y,
			the_pose_now.pose.orientation.z,
			the_pose_now.pose.orientation.w);

	ROS_INFO("The target pose: (%f,%f,%f) ; (%f,%f,%f,%f)",
			target_pose.pose.position.x,
			target_pose.pose.position.y,
			target_pose.pose.position.z,
			target_pose.pose.orientation.x,
			target_pose.pose.orientation.y,
			target_pose.pose.orientation.z,
			target_pose.pose.orientation.w);

	return group.plan(doro_plan);
}


/**
 * Attaches the table obstacle to the planning scene interface.
 */
void DoroManipulation::addTableAsObstacle()
{
	ros::param::set("/plane_extraction_enable", true);

	while(table_position.point.x == table_position.point.y &&
			table_position.point.x == table_position.point.z &&
			table_position.point.x == 0)
	{
		ROS_INFO("Waiting for table position...");
	    sleep(1);
	}

	moveit_msgs::CollisionObject collision_object;
	collision_object.header.frame_id = group.getPlanningFrame();

	/* The id of the object is used to identify it. */
	collision_object.id = "table";

	/* Define a box to add to the world. */
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 0.8;
	primitive.dimensions[1] = 0.8;
	primitive.dimensions[2] = 0.025;

	/* A pose for the box (specified relative to frame_id) */
	geometry_msgs::Pose table_pose;
	table_pose.orientation.w = 1.0;
	table_pose.position.x = table_position.point.x;
	table_pose.position.y = table_position.point.y;
	table_pose.position.z = table_position.point.z;

	collision_object.primitives.push_back(primitive);
	collision_object.primitive_poses.push_back(table_pose);
	collision_object.operation = collision_object.ADD;

	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.push_back(collision_object);

	ps_interface.addCollisionObjects(collision_objects);


	ROS_INFO("THE PLANE WAS ADDED! SEE RVIZ!");
	//std::cout<<"Table eqn:= "<<table_coeffs<<std::endl;
	//std::cout<<"Table position:= "<<table_position<<std::endl;

	ros::param::set("/plane_extraction_enable", false);
}

/**
 * Attaches the object that is going to be picked up as obstacle.
 */
void DoroManipulation::addTargetAsObstacle()
{
	moveit_msgs::CollisionObject collision_object;
	collision_object.header.frame_id = group.getPlanningFrame();

	/* The id of the object is used to identify it. */
	collision_object.id = "pill_box";

	/* Define a box to add to the world. */
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 0.025;
	primitive.dimensions[1] = 0.035;
	primitive.dimensions[2] = 0.080;

	/* A pose for the box (specified relative to frame_id) */
	geometry_msgs::Pose pill_box_pose;
	pill_box_pose.orientation.w = 1.0;
	pill_box_pose.position.x = pose_list.grasp_poses[0].pose.position.x;
	pill_box_pose.position.y = pose_list.grasp_poses[0].pose.position.y;
	pill_box_pose.position.z = pose_list.grasp_poses[0].pose.position.z;

	collision_object.primitives.push_back(primitive);
	collision_object.primitive_poses.push_back(pill_box_pose);
	collision_object.operation = collision_object.ADD;

	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.push_back(collision_object);

	ps_interface.addCollisionObjects(collision_objects);

	ROS_INFO("The box was added as a collision object!");
}

void DoroManipulation::removeTable()
{
	std::vector<std::string> object_ids;
	object_ids.push_back("table");
	ps_interface.removeCollisionObjects(object_ids);
}

void DoroManipulation::removeTarget()
{
	std::vector<std::string> object_ids;
	object_ids.push_back("table");
	ps_interface.removeCollisionObjects(object_ids);
}

void DoroManipulation::attachTarget()
{
	group.attachObject("pill_box");
	ROS_INFO("Pill box is in gripper now.");
}

void DoroManipulation::detachTarget()
{
	group.detachObject("pill_box");
	removeTarget();
	ROS_INFO("Pill box removed.");
}

void DoroManipulation::closeHand(float value)
{
	jaco_msgs::FingerPosition FP;

	FP.Finger_1 = value;
	FP.Finger_2 = value;
	FP.Finger_3 = value;

	finger_pub.publish(FP);
}

void DoroManipulation::openHand()
{
	jaco_msgs::FingerPosition FP;

	FP.Finger_1 = 0.0;
	FP.Finger_2 = 0.0;
	FP.Finger_3 = 0.0;

	finger_pub.publish(FP);
}
