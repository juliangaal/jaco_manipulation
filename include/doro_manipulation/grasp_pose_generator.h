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

#include <std_srvs/Empty.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <doro_manipulation/GenerateGraspPoses.h>

namespace doro_manipulation
{
/**
 * Provides a ROS service to generate a graps pose from a point in 3D.
 * Assumes that the point is within bounds and reachable.
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
     * A listener for the transform information.
     */
    tf::TransformListener *tf_listener_;

protected:

    /**
     * ROS server that returns a grasp pose when object's location is given.
     */
    ros::ServiceServer server_;

    /**
     * Server callback.
     */
    bool serverCB(doro_manipulation::GenerateGraspPosesRequest & _request,
    										doro_manipulation::GenerateGraspPosesResponse & _response);

    /**
     * Convenience function.
     */
    inline float dist3D(const geometry_msgs::Point p1, const geometry_msgs::Point p2);

    /**
     * Convenience function to transform a point to the base_link frame.
     */
    geometry_msgs::PointStamped transformPointToBaseLink(const geometry_msgs::PointStamped& in_pt);

    /**
     * A common nodehandle.
     */
    ros::NodeHandle n_;

    /**
     * The generated poses are stored in these variables.
     */
    geometry_msgs::PoseStamped 	target_pose_top_,
    								target_pose_1_,
    								target_pose_2_,
    								target_pose_3_,
    								hand_pose_;

};
}
#endif /* JACO_HAND_POSE_GENERATOR_H_ */
