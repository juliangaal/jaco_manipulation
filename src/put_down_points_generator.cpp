/*
 * put_down_points_generator.cpp
 *
 *  Created on: Oct 3, 2014
 *      Author: ace
 */

#include <doro_manipulation/put_down_points_generator.h>

namespace doro_manipulation {

PutDownPointsGenerator::PutDownPointsGenerator ()
{
	server_ = nh_.advertiseService ("generate_put_down_points", &PutDownPointsGenerator::serverCB, this);
	clusters_sub_ = nh_.subscribe ("/clusters", 1, &PutDownPointsGenerator::clustersCB, this);
	table_position_sub_ = nh_.subscribe ("/table_position", 1, &PutDownPointsGenerator::tablePositionCB, this);


}
void PutDownPointsGenerator::tablePositionCB(const geometry_msgs::PointStampedConstPtr& table_position)
{
	table_position_ptr_ = geometry_msgs::PointStampedPtr (new geometry_msgs::PointStamped (*table_position) );
}

void PutDownPointsGenerator::clustersCB (const doro_msgs::ClusterArrayConstPtr& clusters)
{
	clusters_ptr_ = doro_msgs::ClusterArrayPtr (new doro_msgs::ClusterArray (*clusters) );
}

bool PutDownPointsGenerator::serverCB (GeneratePutDownPointsRequest& _request, GeneratePutDownPointsResponse& _response)
{
	ros::param::set("/cluster_extraction_enable", true);
	ros::param::set("/plane_extraction_tolerance", 0.4);

	while(!clusters_ptr_ || !table_position_ptr_)
	{
		ROS_INFO("Waiting for clusters and table position...");
		sleep(1);
	}
	ros::param::set("/cluster_extraction_enable", false);
	ros::param::set("/plane_extraction_tolerance", 0.1);

	geometry_msgs::PointStamped target_point[4];

	for(int i = 0; i < 4; i++)
	{
		target_point[i].header.frame_id = table_position_ptr_->header.frame_id;
		target_point[i].point.z = table_position_ptr_->point.z + 0.075;
	}

	// Centre of the table observed.
	target_point[0].point = table_position_ptr_->point;

	// Closer to robot and to the left
	target_point[1].point.x = table_position_ptr_->point.x - 0.2;
	target_point[1].point.y = table_position_ptr_->point.y + 0.2;

	// Closer to robot and to the centre
	target_point[2].point.x = table_position_ptr_->point.x - 0.2;
	target_point[2].point.y = table_position_ptr_->point.y;

	// Closer to robot and to the right
	target_point[3].point.x = table_position_ptr_->point.x - 0.2;
	target_point[3].point.y = table_position_ptr_->point.y - 0.2;

	bool point_safe = true;

	// Check for safe points and select it.
	for(int i = 0; i < 4; i++)
	{
		for(doro_msgs::ClusterArray::_clusters_type::const_iterator iter = clusters_ptr_->clusters.begin(); iter != clusters_ptr_->clusters.end(); iter++)
		{
			if(DIST2D(iter->centroid.point.x, iter->centroid.point.y, target_point[i].point.x, target_point[i].point.y) < 0.075)
			{
				point_safe = false;
				break;
			}
		}

		if(point_safe)
		{
			_response.put_down_points.push_back(target_point[i]);
		}
	}

	clusters_ptr_.reset();
	table_position_ptr_.reset();
	return true;
}

PutDownPointsGenerator::~PutDownPointsGenerator()
{
	clusters_sub_.shutdown();
	table_position_sub_.shutdown();

}

} /* namespace doro_manipulation */
