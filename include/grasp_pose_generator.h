/*
 * grasp_pose_generator.h
 *
 *  Created on: Apr 15, 2014
 *      Author: Chittaranjan Srinivas
 */

#ifndef GRASP_POSE_GENERATOR_H_
#define GRASP_POSE_GENERATOR_H_

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
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
#include "doro_msgs/Clusters.h"


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


/**
 * The class that provides the interface to generate hand poses from the
 * information provided by cylinder_extraction or euclidean_cluster_extraction.
 */
class GraspPoseGenerator
{


public:
	/**
	 * Constructor.
	 */
    GraspPoseGenerator();

    /**
     * Destructor.
     */
    ~GraspPoseGenerator();

    /**
     * Function to tilt the ptu. This should be here because we tilt the ptu before
     * we set the parameter cylinder_extraction_enable.
     */
    void tiltPtu(float value);

    /**
     * A listener for the transform information.
     */
    tf::TransformListener *tf_listener;

protected:
    /**
     * A callback function for the clusters.
     */
    void clustersCB(const doro_msgs::ClustersConstPtr& _clusters);

    /**
     * Convenience function.
     */
    inline float dist3D(const geometry_msgs::Point p1, const geometry_msgs::Point p2);

    /**
     * Convenience function to transform a point to the base_link frame.
     */
    geometry_msgs::PointStamped transformPointToBaseLink(const geometry_msgs::PointStamped& in_pt);

    ros::NodeHandle n_;

    ros::Subscriber clusters_sub_;

    ros::Publisher pub_grasp_poses_;
    ros::Publisher pub_ptu_;

    tf::Vector3 cluster_position[4];
    tf::Vector3 old_p_;
    double avg_count_;

    ros::Publisher pub_goal_;


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
