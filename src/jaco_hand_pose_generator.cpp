/*
 * jaco_hand_pose_generator.cpp
 *
 *  Created on: Mar 19, 2014
 *      Author: ace
 */

#include "jaco_hand_pose_generator.h"

JacoHandPoseGenerator::JacoHandPoseGenerator()
{
    ros::NodeHandle np;

    tf_ = new tf::TransformListener(np,ros::Duration(10));

    sub_model_cylinder_ =
        n_.subscribe("/model_cylinder", 10, &JacoHandPoseGenerator::modelCylinderCB, this);

    //sub_proj_ =
     //   n_.subscribe("/projection", 10, &JacoHandPoseGenerator::projCB, this);

    pub_goal_ =
    	n_.advertise<geometry_msgs::PoseStamped>("/cylinder_pose", 10);

    pub_grasp_poses_ = n_.advertise<doro_msgs::GraspPoses>("/grasp_poses", 10);

    pub_ptu_ = n_.advertise<sensor_msgs::JointState>("/ptu/cmd", 2);


    // cyl_height_ = 0.0;
    // old_cyl_height_ = 0.0;
    avg_count_ = 0.0;

    _p_.setValue(0.0,0.0,0.0);
    _cyl_.setValue(0.0,0.0,0.0);

   // max_avg_ = 0.0;
   // min_avg_ = 0.0;


    old_p_.setValue(0.0,0.0,0.0);
    old_cyl_.setValue(0.0,0.0,0.0);

    tiltPtu(-0.5);

    ros::param::set("/cylinder_extraction_enable", true);

}

void JacoHandPoseGenerator::tiltPtu(float value)
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

JacoHandPoseGenerator::~JacoHandPoseGenerator()
{
	tiltPtu(0.0);

	ROS_INFO("JacoHandPoseGenerator Shutting down...");
	sub_model_cylinder_.shutdown();
	sub_proj_.shutdown();

	ros::param::set("/cylinder_extraction_enable", false);

	if(tf_)
		delete tf_;
}

/*
void JacoHandPoseGenerator::projCB(const PointCloud::ConstPtr& msg)
{

        if (msg->points.size() < 10)
        {
            //ROS_DEBUG("Less than 10 points in projection.");
            return;
        }

        // Find out the cylinder's bounds by projecting every point
        // on the model's axis.
        double min = 1;
        double max = -1;

        BOOST_FOREACH(const pcl::PointXYZ& pt, msg->points)
        {
            tf::Vector3 p = tf::Vector3(pt.x, pt.y, pt.z) - old_p_;
            double h = p.dot(old_cyl_);

            if (h > max)
                max = h;
            if (h < min)
                min = h;
        }

        cyl_height_ = max - min;
        cyl_height_ = ( (old_cyl_height_*avg_count_) + cyl_height_) / (avg_count_ + 1.0);
        old_cyl_height_ = cyl_height_;

        //cyl_height_ = old_cyl_height_ +
          //  (((max - min) - old_cyl_height_) / avg_count_);
        //old_cyl_height_ = cyl_height_;

}
*/

void JacoHandPoseGenerator::modelCylinderCB(const pcl_msgs::ModelCoefficientsConstPtr& msg)
{

        if (msg->values.size() < 7)
        {
            ROS_DEBUG("Not enough coefficients to be a cylinder: %i.", (int)msg->values.size());
            return;
        }


        // 1. Create the target pose.

        // Cylinder structure: {{point_on_axis}, {axis_direction}, radius}.
        // The axis direction is normalized.
        //
        // TODO: Figure out where the point is relative to the cylinder,
        // e.g the base, CoG, ...

        // Cylinder origin, seems to be CoG.
        tf::Vector3 p(
            msg->values[0],
            msg->values[1],
            msg->values[2]);

        // Cylinder vertical axis.
        tf::Vector3 cyl(
            msg->values[3],
            msg->values[4],
            msg->values[5]);

        // TODO: Make sure it points to the sky (needs a reference).
        // The test is currently baked knowing the fact that the Y axis
        // of the camera points to the ground.
        if (cyl.y() > 0)
            cyl *= -1.0;

        // Cummulative average on the position, axis and height.
        /*
        avg_count_ += 1.0;
        p = old_p_ + ((p - old_p_) / avg_count_);
        cyl = old_cyl_ + ((cyl - old_cyl_) / avg_count_);
        cyl.normalize();
        old_p_ = p;
        old_cyl_ = cyl;
 	 	*/

        /* *********************************** DEBUG **************************************** *
        ROS_INFO("****BEFORE****, , , CYL.X: %f, CYL.Y: %f, CYL.Z: %f", cyl.x(), cyl.y(), cyl.z());
        ROS_INFO("****BEFORE****, , , P.X: %f, P.Y: %f, P.Z: %f", p.x(), p.y(), p.z());
         * *********************************** DEBUG **************************************** */

        if(avg_count_ > 0)
        {
        	if((_p_ - p).length() > 0.25)
        	{
        		_p_ = p;
        		_cyl_ = cyl;
        		//ROS_INFO("GARBAGE1!");
        		return;
        	}
        }
        else
        {
        	if(p.y() < -1.00 || p.y() > 1.00)
        		return;
        }

        _p_ = p;
        _cyl_ = cyl;

        if(avg_count_ > 9.00)
        {
        	avg_count_ = 1.00;
        }

        else
        {
        	// Simple formula for mean
        	p = ( (old_p_*avg_count_) + p) / (avg_count_ + 1.0);
        	cyl = ( (old_cyl_*avg_count_) + cyl) / (avg_count_ + 1.0);

        //ROS_INFO("ITER: %f", avg_count_);

        	avg_count_+=1.0;
        }



        /* *********************************** DEBUG **************************************** *
        ROS_INFO("****AFTER AVERAGE****, , , CYL.X: %f, CYL.Y: %f, CYL.Z: %f", cyl.x(), cyl.y(), cyl.z());
        ROS_INFO("****AFTER AVERAGE****, , , P.X: %f, P.Y: %f, P.Z: %f", p.x(), p.y(), p.z());
         * *********************************** DEBUG **************************************** */

        cyl.normalize();

        old_p_ = p;
        old_cyl_ = cyl;

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


         // The Vectors for the top grasp
         tf::Vector3 view_top = p.normalized();
         tf::Vector3 oz_top = view_top;
         tf::Vector3 ox_top = cyl.cross(view_top);

         oz_top = rotatex*rotatex*oz_top;
         tf::Vector3 oy_top = oz_top.cross(ox_top);
         ox_top = rotate10z * rotate10z * ox_top;
         oy_top = rotate10z * rotate10z * oy_top;

         // Simple grasp vectors
         tf::Vector3 view = p.normalized();
         tf::Vector3 oy_1 = cyl;
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

         // A 25 degree rotation about the openni (xtion) optical frame's y-axis
         oz_2 = rotate25y * oz_1;
         ox_2 = rotate25y * ox_1;
         oy_2 = oy_1;

         // A 50 degree rotation about the openni (xtion) optical frame's y-axis

         oz_3 = rotate25y * oz_2;
         ox_3 = rotate25y * ox_2;
         oy_3 = oy_2;

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
        target_pose_top_.header =  msg->header;
        target_pose_1_.header = msg->header;
        target_pose_2_.header = msg->header;
        target_pose_3_.header = msg->header;

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


        // ALE bug. Jaco has problem with complicated quaternion. If we send them, the arm stucks.
        // let's put fixed quaternion and convert the pose into the arm frame

        geometry_msgs::PoseStamped target_goal_in_base_frame;

        // Let us freakin create the message variable
        doro_msgs::GraspPosesPtr pose_list (new doro_msgs::GraspPoses);

        pose_list->grasp_poses.resize(4);
        pose_list->pregrasp_poses.resize(4);

        try {
            tf_->waitForTransform("/base_link", msg->header.frame_id, msg->header.stamp, ros::Duration(1));


// Top grasp
        	tf_->transformPose("/base_link", target_pose_top_, target_goal_in_base_frame); // transf target goal in hand frame

        	// First the grasp pose. Then the pregrasp pose
        	target_goal_in_base_frame.pose.position.z += 0.02;
        	pose_list->grasp_poses[3] = target_goal_in_base_frame;
        	target_goal_in_base_frame.pose.position.z += 0.03;
        	pose_list->pregrasp_poses[3] = target_goal_in_base_frame;

// First simple grasp
        	tf_->transformPose("/base_link", target_pose_1_, target_goal_in_base_frame); // transf target goal in hand frame

        	// First the grasp pose. Then the pregrasp pose
        	target_goal_in_base_frame.pose.position.x += 0.02;
        	pose_list->grasp_poses[1] = target_goal_in_base_frame;
        	target_goal_in_base_frame.pose.position.x -= 0.05;
        	pose_list->pregrasp_poses[1] = target_goal_in_base_frame;

// Second simple grasp
        	tf_->transformPose("/base_link", target_pose_2_, target_goal_in_base_frame); // transf target goal in hand frame

        	// First the grasp pose. Then the pregrasp pose
        	target_goal_in_base_frame.pose.position.x += 0.015;
        	target_goal_in_base_frame.pose.position.y += 0.015;
        	pose_list->grasp_poses[2] = target_goal_in_base_frame;
        	target_goal_in_base_frame.pose.position.x -= 0.045;
        	target_goal_in_base_frame.pose.position.y -= 0.045;
        	pose_list->pregrasp_poses[2] = target_goal_in_base_frame;

// Third simple grasp
        	tf_->transformPose("/base_link", target_pose_3_, target_goal_in_base_frame); // transf target goal in hand frame

        	// First the grasp pose. Then the pregrasp pose
        	target_goal_in_base_frame.pose.position.x += 0.020;
        	target_goal_in_base_frame.pose.position.y += 0.020;
        	pose_list->grasp_poses[0] = target_goal_in_base_frame;
        	target_goal_in_base_frame.pose.position.x -= 0.045;
        	target_goal_in_base_frame.pose.position.y -= 0.055;
        	pose_list->pregrasp_poses[0] = target_goal_in_base_frame;

        } catch (tf::TransformException& ex)
        {
            ROS_WARN("Can't transform target pose. Reason %s", ex.what());
            return;
        }

        // Set the number and despatch
        pose_list->number_of_poses = pose_list->grasp_poses.size();

        pub_grasp_poses_.publish(pose_list);
        pub_goal_.publish(pose_list->grasp_poses[0]);

}

float JacoHandPoseGenerator::dist3D(const geometry_msgs::Point p1, const geometry_msgs::Point p2)
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
	JacoHandPoseGenerator JHPG;
	ros::spin();
}
*/
