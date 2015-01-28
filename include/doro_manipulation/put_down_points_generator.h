/*
 * put_down_points_generator.h
 *
 *  Created on: Oct 3, 2014
 *      Author: ace
 */

#ifndef PUT_DOWN_POINTS_GENERATOR_H_
#define PUT_DOWN_POINTS_GENERATOR_H_

#include <doro_msgs/ClusterArray.h>
#include <geometry_msgs/PointStamped.h>
#include <doro_manipulation/GeneratePutDownPoints.h>
#include <ros/ros.h>

#include <doro_manipulation/common.h>

namespace doro_manipulation {

/**
 * \brief Provides a ROS service to look at a table and generate safe put down points.
 * Assumes that there is a table in front.
 * Ideally called after a Look.
 */
class PutDownPointsGenerator
{
protected:
	/**
	 * A common nodehandle.
	 */
	ros::NodeHandle nh_;

    /**
     * ROS server that returns safe put down points after looking at clusters.
     */
    ros::ServiceServer server_;

	/**
	 * Server callback.
	 */
	bool serverCB(GeneratePutDownPointsRequest & _request,
			GeneratePutDownPointsResponse & _response);

	/**
	 * Clusters shared_ptr to hold latest value.
	 */
	doro_msgs::ClusterArrayPtr clusters_ptr_;

	/**
	 * ROS subscriber to listen to clusters.
	 */
	ros::Subscriber clusters_sub_;

	/**
	 * Callback function for clusters.
	 */
	void clustersCB(const doro_msgs::ClusterArrayConstPtr& clusters);

	/**
	 * A shared_ptr to hold the latest table_position.
	 */
	geometry_msgs::PointStampedPtr table_position_ptr_;

	/**
	 * ROS subscriber to listen on table_position.
	 */
	ros::Subscriber table_position_sub_;

	/**
	 * A callback function for the table_position.
	 */
	void tablePositionCB(const geometry_msgs::PointStampedConstPtr& table_position);

public:
	/**
	 * Constructor.
	 */
	PutDownPointsGenerator();
	virtual ~PutDownPointsGenerator();
};

} /* namespace doro_manipulation */

#endif /* PUT_DOWN_POINTS_GENERATOR_H_ */
