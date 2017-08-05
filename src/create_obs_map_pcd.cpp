#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_ros/point_cloud.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <math.h>

using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr obs_cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);

void obs_map_callback(nav_msgs::OccupancyGrid map){
	pcl::PointXYZ obstacle_point;
	
	int map_size = map.info.width * map.info.height;

	cout<<"map_size : "<<map_size<<endl;

	for(int i = 0;i<map_size; i++){
		// cout<<"count["<<i<<"] : "<<count[i]<<endl;
		if(map.data[i] == 100){
			obstacle_point.x = (i % map.info.width) * map.info.resolution + map.info.origin.position.x;
			obstacle_point.y = int(i / map.info.width) * map.info.resolution + map.info.origin.position.y;
			obstacle_point.z = 0;

			obs_cloud->push_back(obstacle_point);				
		}
	}
	pcl::VoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
	approximate_voxel_filter.setLeafSize(0.3, 0.3, 0.3);
	approximate_voxel_filter.setInputCloud(obs_cloud);
	approximate_voxel_filter.filter(*filtered_cloud);
	cout<<"Raw Cloud points : "<<obs_cloud->size()<<endl;
	cout<<"Filtered cloud points : "<<filtered_cloud->size()<<endl;
	
	// pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	// sor.setInputCloud(obs_cloud);
	// sor.setMeanK(50);
	// sor.setStddevMulThresh(5.0);
	// sor.filter(*filtered_cloud);

	// pcl::io::savePCDFile("obs_cloud_data_curvature.pcd", *obs_cloud);
	// pcl::io::savePCDFile("obs_cloud_data_tsukuba.pcd", *obs_cloud);
	// pcl::io::savePCDFile("obs_cloud_data_ikuta.pcd", *obs_cloud);
	// pcl::io::savePCDFile("obs_cloud_data_ikuta_filtered.pcd", *filtered_cloud);
	// pcl::io::savePCDFile("obs_cloud_data_tsukuba_filtered.pcd", *filtered_cloud);
	
	pcl::io::savePCDFile("obs_cloud_data_robosym2016.pcd", *obs_cloud);
	// pcl::io::savePCDFile("obs_cloud_data_robosym2016_filtered.pcd", *filtered_cloud);
	cout<<"PCD File save!!!!!!"<<endl;

}



float rotation_x(float x, float y, float theta){
	float x_r;
	// if(theta >= 0){
	x_r = cos(theta) * x - sin(theta) * y;
	// }
	// else{
		// x_r = cos(theta) * x - sin(theta) * y;
	// }
	return x_r;
}

float rotation_y(float x, float y, float theta){
	float y_r;
	// if(theta >= 0){
	y_r = sin(theta) * x + cos(theta) * y;
	// }
	// else{
		// y_r = -1 * sin(theta) * x + cos(theta) * y;
	// }
	return y_r;
}

int main (int argc, char** argv){
	// Initialize ROS
	ros::init (argc, argv, "global_obstacle_map_creater");
	ros::NodeHandle n;

	ros::Subscriber sub1 = n.subscribe("/dilated_obs_map", 1 , obs_map_callback);

	
	ros::spin();

	return 0;
}
