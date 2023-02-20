#pragma once
#ifndef _GK_MINDIST
#define _GK_MINDIST


#include <math.h>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "gatekeeper/MinDist.h"

// time sync
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

class GKMinDistSrv
{

public:
	// constructor
	GKMinDistSrv(ros::NodeHandle nh);

private:
	ros::NodeHandle nh_;

	ros::ServiceServer service_;

	ros::Subscriber sub_pc_unknown_;
	ros::Subscriber sub_pc_occupied_;

	bool minDist(gatekeeper::MinDist::Request & req, gatekeeper::MinDist::Response &res);
	
	void mapCB(const sensor_msgs::PointCloud2::ConstPtr& pcl2ptr_msg,
             const sensor_msgs::PointCloud2::ConstPtr& pcl2ptr_msg2);  // Callback for the occupancy pcloud
  	void unkCB(const sensor_msgs::PointCloud2ConstPtr& pcl2ptr_msg);     // Callback for the unkown pcloud
  	void pclCB(const sensor_msgs::PointCloud2ConstPtr& pcl2ptr_msg);

 	message_filters::Subscriber<sensor_msgs::PointCloud2> occup_grid_sub_;
  	message_filters::Subscriber<sensor_msgs::PointCloud2> unknown_grid_sub_;
  	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
  	typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  	boost::shared_ptr<Sync> sync_;

	// variables
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;


}; // class GKMinDistSrv





#endif //_GK_MINDIST

