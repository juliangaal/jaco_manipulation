/*
 * grasp_pose_generator.cpp
 *
 *  Created on: Apr 15, 2014
 *      Author: ace
 */

#include <grasp_pose_generator.h>

GraspPoseGenerator::GraspPoseGenerator()
{
    ros::NodeHandle np;

    tf_listener = new tf::TransformListener(np,ros::Duration(10));

    pub_goal_ =
    	n_.advertise<geometry_msgs::PoseStamped>("/cluster_pose", 10);

    pub_grasp_poses_ = n_.advertise<doro_msgs::GraspPoses>("/grasp_poses", 10);

    pub_ptu_ = n_.advertise<sensor_msgs::JointState>("/ptu/cmd", 2);

    clusters_sub_ = n_.subscribe("/clusters", 1, &GraspPoseGenerator::clustersCB, this);

    for(int i = 0; i < 4; i++)
    	cluster_position[i].setValue(0.0,0.0,0.0);

    old_p_.setValue(0.0,0.0,0.0);

    avg_count_ = 0.0;
    tiltPtu(-0.5);

    ros::param::set("/cluster_extraction_enable", true);

}

void GraspPoseGenerator::tiltPtu(float value)
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
	pub_ptu_.publish(ptu_msg);
	sleep(1);

	ROS_INFO("TILTED PTU");
}

GraspPoseGenerator::~GraspPoseGenerator()
{
	tiltPtu(0.0);

	ROS_INFO("GraspPoseGenerator Shutting down...");
	clusters_sub_.shutdown();

	ros::param::set("/cluster_extraction_enable", false);

	if(tf_listener)
		delete tf_listener;
}

geometry_msgs::PointStamped GraspPoseGenerator::transformPointToBaseLink(const geometry_msgs::PointStamped& in_pt)
{
	geometry_msgs::PointStamped out_pt;

	try
	{
		tf_listener->waitForTransform("base_link", in_pt.header.frame_id, in_pt.header.stamp, ros::Duration(1));
		tf_listener->transformPoint("base_link", in_pt, out_pt);
	}
	catch(tf::TransformException& exception)
	{
		ROS_INFO("Transform failed. Why? - %s", exception.what());
	}

	return out_pt;
}

void GraspPoseGenerator::clustersCB(const doro_msgs::ClustersConstPtr& _clusters)
{
		if(_clusters->cluster_centroids.size() == 0)
		{
			ROS_INFO("In GraspPoseGenerator: No clusters in the message.");
			return;
		}

	 	tf::Vector3 p (
	 					_clusters->cluster_centroids[0].point.x,
	 					_clusters->cluster_centroids[0].point.y,
	 					_clusters->cluster_centroids[0].point.z + 0.03
	 				   );

	 	if(p.z() > 2.0 || p.z() < 0.5)
	 	{
	 		ROS_INFO("In GraspPoseGenerator: Out of bounds value...");
	 		return;
	 	}


	 	if(avg_count_ > 9.00)
	 	{
	 		avg_count_ = 1.00;
	 	}
	 	else
	 	{
	 		//ROS_INFO("Averaging...");
	 		p = ( (old_p_*avg_count_) + p) / (avg_count_ + 1.0);

	 		avg_count_+=1.0;
	 	}

        old_p_ = p;

        geometry_msgs::PointStamped pt_o; // The origin of the openni frame in the base_link frame.

        pt_o.header.stamp = ros::Time::now();
        pt_o.header.frame_id = "xtion_camera_depth_optical_frame";
        pt_o.point.x = 0.0;
        pt_o.point.y = 0.0;
        pt_o.point.z = 0.0;

        pt_o = transformPointToBaseLink(pt_o);

        // The view vector is simply (p - pt_o)

        tf::Vector3 view;
        view.setValue (
        		p.x() - pt_o.point.x,
        		p.y() - pt_o.point.y,
        		p.z() - pt_o.point.z);

        // Orientation:
        //  - X points to the thumb.
        //  - Y is perpendicular to the grasp plane.
        //  - Z is the wrist rotation axis, and should point outward
        //    the camera.

        // Let's jiggle it a bit
        tf::Matrix3x3 rotate45y (0.707, 0, -0.707, 0, 1, 0, 0.707, 0, 0.707);
        tf::Matrix3x3 rotate50y (0.64278, 0, -0.766, 0, 1, 0, 0.766, 0, 0.64278);
        tf::Matrix3x3 rotate25y (0.906, 0, -0.423, 0, 1, 0, 0.423, 0, 0.906);
        tf::Matrix3x3 rotate10z (0.9848, 0.17365, 0, -0.17365, 0.9848, 0, 0, 0, 1);
        tf::Matrix3x3 rotatex (1, 0, 0, 0, 0.9848, 0.17365, 0, -0.17365, 0.9848);

        tf::Matrix3x3 rotatey (0.9848, 0, 0.17365, 0, 1, 0, -0.17365, 0, 0.9848);
        tf::Matrix3x3 rotate25z (0.906, -0.423, 0, 0.423, 0.906, 0, 0, 0, 1);



         // The Vectors for the top grasp
         tf::Vector3 abs_z (0.0, 0.0, 1.0);
         tf::Vector3 oz_top = view;
         tf::Vector3 ox_top = abs_z.cross(view);

         oz_top = rotatey*oz_top;
         tf::Vector3 oy_top = oz_top.cross(ox_top);

         // Simple grasp vectors
         tf::Vector3 oy_1 = abs_z;
         tf::Vector3 ox_1 = (oy_1.cross(view).normalized());
         tf::Vector3 oz_1 = ox_1.cross(oy_1);

         // Second simple grasp
         tf::Vector3 ox_2;
         tf::Vector3 oy_2;
         tf::Vector3 oz_2;

         // Third
         tf::Vector3 ox_3;
         tf::Vector3 oy_3;
         tf::Vector3 oz_3;

         // A 25 degree rotation about the global z
         oz_2 = rotate25z * oz_1;
         ox_2 = rotate25z * ox_1;
         oy_2 = oy_1;

         // A 50 degree rotation about the global z

         oz_3 = rotate25z * oz_2;
         ox_3 = rotate25z * ox_2;
         oy_3 = oy_2;

         /*
         // This has to be done after all the rotations in y. This is a fine tuning.
         // Set 1
         ox_1 = rotate10z * rotate10z * ox_1;
         oy_1 = rotate10z * rotate10z * oy_1;
         oz_1 = rotate10z * rotate10z * oz_1;

         // Set 2
         ox_2 = rotate10z * rotate10z * ox_2;
         oy_2 = rotate10z * rotate10z * oy_2;
         oz_2 = rotate10z * rotate10z * oz_2;

         // Set 3
         ox_3 = rotate10z * rotate10z * ox_3;
         oy_3 = rotate10z * rotate10z * oy_3;
         oz_3 = rotate10z * rotate10z * oz_3;
*/

        // ROS_INFO("ox. X: %f, Y: %f, Z: %f", ox.x(), ox.y(), ox.z());
        // ROS_INFO("oy. X: %f, Y: %f, Z: %f", oy.x(), oy.y(), oy.z());
        // ROS_INFO("oz. X: %f, Y: %f, Z: %f", oz.x(), oz.y(), oz.z());

        // Convert this into a quaternion from the basis matrix.
        tf::Matrix3x3 basis_top(
            ox_top.x(), oy_top.x(), oz_top.x(),
            ox_top.y(), oy_top.y(), oz_top.y(),
            ox_top.z(), oy_top.z(), oz_top.z() );

        tf::Matrix3x3 basis_1(
                    ox_1.x(), oy_1.x(), oz_1.x(),
                    ox_1.y(), oy_1.y(), oz_1.y(),
                    ox_1.z(), oy_1.z(), oz_1.z() );

        tf::Matrix3x3 basis_2(
                    ox_2.x(), oy_2.x(), oz_2.x(),
                    ox_2.y(), oy_2.y(), oz_2.y(),
                    ox_2.z(), oy_2.z(), oz_2.z() );

        tf::Matrix3x3 basis_3(
                    ox_3.x(), oy_3.x(), oz_3.x(),
                    ox_3.y(), oy_3.y(), oz_3.y(),
                    ox_3.z(), oy_3.z(), oz_3.z() );

        tf::Vector3 target_p = p;

        // Verbose - bad programming - but this should be enough for the simple case

        // Header assignment
        target_pose_top_.header =  _clusters->cluster_centroids[0].header ;
        target_pose_1_.header = _clusters->cluster_centroids[0].header;
        target_pose_2_.header = _clusters->cluster_centroids[0].header;
        target_pose_3_.header = _clusters->cluster_centroids[0].header;

        geometry_msgs::Point _point;

        tf::pointTFToMsg(target_p, _point);

        target_pose_top_.pose.position = _point;
        target_pose_1_.pose.position = _point;
        target_pose_2_.pose.position = _point;
        target_pose_3_.pose.position = _point;

        tf::Quaternion q_1, q_2, q_3, q_top;

        // Top
        basis_top.getRotation(q_top);
        q_top = q_top.normalized();
        tf::quaternionTFToMsg(q_top, target_pose_top_.pose.orientation);

        // 1
        basis_1.getRotation(q_1);
        q_1 = q_1.normalized();
        tf::quaternionTFToMsg(q_1, target_pose_1_.pose.orientation);

        // 2
        basis_2.getRotation(q_2);
        q_2 = q_2.normalized();
        tf::quaternionTFToMsg(q_2, target_pose_2_.pose.orientation);

        // 3
        basis_3.getRotation(q_3);
        q_3 = q_3.normalized();
        tf::quaternionTFToMsg(q_3, target_pose_3_.pose.orientation);


        geometry_msgs::PoseStamped target_goal_in_base_frame;

        // Let us freakin create the message variable
        doro_msgs::GraspPosesPtr pose_list (new doro_msgs::GraspPoses);

        pose_list->grasp_poses.resize(4);
        pose_list->pregrasp_poses.resize(4);


        	// First the grasp pose. Then the pregrasp pose
            target_pose_top_.pose.position.z += 0.02;
            target_pose_top_.pose.position.x += 0.025;
        	pose_list->grasp_poses[3] = target_pose_top_;
        	target_pose_top_.pose.position.z += 0.03;
        	pose_list->pregrasp_poses[3] = target_pose_top_;


        	// First the grasp pose. Then the pregrasp pose
        	target_pose_1_.pose.position.x += 0.020;
        	pose_list->grasp_poses[1] = target_pose_1_;
        	target_pose_1_.pose.position.x -= 0.07;
        	pose_list->pregrasp_poses[1] = target_pose_1_;

        	// First the grasp pose. Then the pregrasp pose
        	target_pose_2_.pose.position.x += 0.020;
        	target_pose_2_.pose.position.y += 0.020;
        	pose_list->grasp_poses[2] = target_pose_2_;
        	target_pose_2_.pose.position.x -= 0.070;
        	target_pose_2_.pose.position.y -= 0.070;
        	pose_list->pregrasp_poses[2] = target_pose_2_;

        	// First the grasp pose. Then the pregrasp pose
        	target_pose_3_.pose.position.x += 0.022;
        	target_pose_3_.pose.position.y += 0.022;
        	pose_list->grasp_poses[0] = target_pose_3_;
        	target_pose_3_.pose.position.x -= 0.070;
        	target_pose_3_.pose.position.y -= 0.070;
        	pose_list->pregrasp_poses[0] = target_pose_3_;

        // Set the number and despatch
        pose_list->number_of_poses = pose_list->grasp_poses.size();

        pub_grasp_poses_.publish(pose_list);
        pub_goal_.publish(pose_list->grasp_poses[0]);

}

float GraspPoseGenerator::dist3D(const geometry_msgs::Point p1, const geometry_msgs::Point p2)
{

	float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    float dz = p2.z - p1.z;

    return sqrt((float)(dx * dx + dy * dy + dz * dz));
}

/*
int main(int argn, char* args[])
{
	ros::init(argn, args, "hand_pose_generator");
	GraspPoseGenerator GPG;
	ros::spin();
}*/

