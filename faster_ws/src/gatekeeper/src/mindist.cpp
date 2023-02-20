
#include "ros/ros.h"
#include "gatekeeper/MinDist.h"
#include "mindist.hpp"
#include <sensor_msgs/point_cloud_conversion.h>
#include <limits>


GKMinDistSrv::GKMinDistSrv(ros::NodeHandle nh)
	:nh_(nh)
{
	
	// Subscribers
	occup_grid_sub_.subscribe(nh_, "/SQ01s/global_mapper_ros/occupancy_grid", 1);
	unknown_grid_sub_.subscribe(nh_, "/SQ01s/global_mapper_ros/unknown_grid", 1);
	sync_.reset(new Sync(MySyncPolicy(1), occup_grid_sub_, unknown_grid_sub_));
	sync_->registerCallback(boost::bind(&GKMinDistSrv::mapCB, this, _1, _2));

	// Service
	service_ = nh.advertiseService("minDist", minDist);

}

void GKMinDistSrv::mapCB(const sensor_msgs::PointCloud2::ConstPtr& pcl2ptr_map_ros,
                      const sensor_msgs::PointCloud2::ConstPtr& pcl2ptr_unk_ros)
{
  // Occupied Space Point Cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr_map(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*pcl2ptr_map_ros, *pclptr_map);
  // Unknown Space Point Cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr_unk(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*pcl2ptr_unk_ros, *pclptr_unk);

  // faster_ptr_->updateMap(pclptr_map, pclptr_unk);
  

  std::cout << "Received point clouds!!" << std::endl;


  // combine them into a single pcl
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud += *pclptr_map;
  cloud += *pclptr_unk;

  // make it into a kdtree
  kdtree_.setInputCloud (cloud);
}



bool GKMinDistSrv::minDist(gatekeeper::MinDist::Request &req, 
		gatekeeper::MinDist::Response &res){

	double minDistSq = std::numeric_limits<double>::infinity();
	
	for (auto &p : req.path){
		//TODO(dev): optimize to prevent checking all points - jump to search next point in circle
		std::cout  << "CHECKING POINT: " << p.x << ", " << p.y << ", " << p.z << std::endl;

		pcl::PointXYZ searchPoint;
		searchPoint.x = p.x;
		searchPoint.y = p.y;
		searchPoint.z = p.z;

		int K = 1;

		std::vector<int> pointIdxKNNSearch(K);
		std::vector<float> pointKNNSquaredDistance(K);
		
		if ( kdtree_.nearestKSearch (searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0 )
		{
			
			minDistSq = std::min(minDistSq, pointKNNSquaredDistance[0]);
		}
	}
	
	res.minDist = std::sqrt(minDistSq);
	return true;
}

// 
// int main(int argc, char** argv){
// 
// 	ros::init(argc, argv, "minDist_server");
// 	ros::NodeHandle nh;
// 
// 
// 
// 	ros::spin();
// 
// 
// 	return 0;
// 
// }
// 
// 



int main(int argc, char **argv)
{
  // Initializes ROS, and sets up a node
  ros::init(argc, argv, "gk_minDist_srv");
  ros::NodeHandle nh("~");
  // ros::CallbackQueue custom_queue1;
  // ros::CallbackQueue custom_queue2;
  // nh.setCallbackQueue(&custom_queue1);
  // nh.setCallbackQueue(&custom_queue2);

  GKMinDistSrv GKMinDistSrv(nh);

  // ros::AsyncSpinner spinner1(0, &custom_queue1);
  // ros::AsyncSpinner spinner2(0, &custom_queue2);
  // spinner1.start();  // start spinner of the custom queue 1
  // spinner2.start();  // start spinner of the custom queue 2

  ros::spin();  // spin the normal queue

  // ros::AsyncSpinner spinner(0);  // 0 means # of threads= # of CPU cores

  ros::waitForShutdown();
  return 0;
}
