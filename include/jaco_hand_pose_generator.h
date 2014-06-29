/*
 * jaco_hand_pose_generator.h
 *
 *  Created on: Mar 19, 2014
 *      Author: ace
 */

#ifndef JACO_HAND_POSE_GENERATOR_H_
#define JACO_HAND_POSE_GENERATOR_H_

#include <geometry_msgs/PoseStamped.h>
#include <pcl/ModelCoefficients.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <std_srvs/Empty.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "doro_msgs/GraspPoses.h"


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

/**
 * The class that provides the interface to generate hand poses from the
 * information provided by cylinder_extraction or euclidean_cluster_extraction.
 */
class JacoHandPoseGenerator
{


public:
	/**
	 * Constructor.
	 */
    JacoHandPoseGenerator();

    /**
     * Destructor.
     */
    ~JacoHandPoseGenerator();

    /**
     * Function to tilt the ptu. This should be here because we tilt the ptu before
     * we set the parameter cylinder_extraction_enable.
     */
    void tiltPtu(float value);

    /**
     * A listener for the transform information.
     */
    tf::TransformListener *tf_;

private:
    /**
     * Callback for the point cloud of the extracted cylinder --- Later for the extracted cluster.
     */
    void projCB(const PointCloud::ConstPtr& msg);

    /**
     * Model coefficients for the cylinder. Centroid and the vector representing the axis.
     */
    void modelCylinderCB(const pcl_msgs::ModelCoefficientsConstPtr& msg);

    /**
     * Model cofficients for the plane (ax + by + cz - d = 0). Typically a, b, c, d.
     * Not used currently since the equation of the plane is in the openni optical frame.
     */
    void modelPlaneCB(const pcl_msgs::ModelCoefficientsConstPtr& msg);

    /**
     * Convenience function.
     */
    inline float dist3D(const geometry_msgs::Point p1, const geometry_msgs::Point p2);

    ros::NodeHandle n_;

    ros::Subscriber sub_model_cylinder_;
    //ros::Subscriber sub_model_plane_;
    ros::Subscriber sub_proj_;

    ros::Publisher pub_grasp_poses_;
    ros::Publisher pub_goal_;
    ros::Publisher pub_ptu_;

    tf::Vector3 old_p_;
    tf::Vector3 old_cyl_;

    tf::Vector3 _p_;
    tf::Vector3 _cyl_;

    double cyl_height_;
    double old_cyl_height_;
    double avg_count_;

    double max_avg_;
    double min_avg_;

    /**
     * The generated poses are stored in these variables.
     */
    geometry_msgs::PoseStamped 	target_pose_top_,
    								target_pose_1_,
    								target_pose_2_,
    								target_pose_3_,
    								hand_pose_;

};

#endif /* JACO_HAND_POSE_GENERATOR_H_ */
