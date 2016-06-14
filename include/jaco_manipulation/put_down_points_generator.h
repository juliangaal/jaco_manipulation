/*
  Copyright (C) 2015  Chittaranjan Srinivas Swaminathan

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>
  
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
