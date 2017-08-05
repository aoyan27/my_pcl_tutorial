#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Odometry.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <iostream>
#include <math.h>
#include <time.h>
#include <stdio.h>

using namespace std;

nav_msgs::OccupancyGrid real_map;
nav_msgs::OccupancyGrid static_map;

pcl::PointCloud<pcl::PointXYZ>::Ptr real_points (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr static_points (new pcl::PointCloud<pcl::PointXYZ>);

sensor_msgs::PointCloud2 vis_real_points;
sensor_msgs::PointCloud2 vis_static_points;
sensor_msgs::PointCloud2 vis_output_points;

nav_msgs::Odometry odom;

bool real_sub_flag = false;
bool static_sub_flag = false;
bool odom_sub_flag = false;

void lcl_callback(nav_msgs::Odometry msg){
	odom = msg;
	// cout<<"odom.pose.position.x : "<<odom.pose.pose.position.x<<endl;
	// cout<<"odom.pose.position.y : "<<odom.pose.pose.position.y<<endl;
	// cout<<"odom.pose.position.z : "<<odom.pose.pose.position.z<<endl;
	// cout<<"odom.pose.orientation.z : "<< odom.pose.pose.orientation.z<<endl;

	odom_sub_flag = true;
}

void RealLocalMap_callback(nav_msgs::OccupancyGrid msg){
	real_map = msg;
	size_t map_size = msg.data.size();

	real_points->clear();
	for(size_t i=0;i<map_size;i++){
		if(msg.data[i] != 0){
			pcl::PointXYZ temp_point;
			temp_point.x = (i % msg.info.width) * msg.info.resolution + msg.info.origin.position.x;
			temp_point.y = (int)(i / msg.info.width) * msg.info.resolution + msg.info.origin.position.y;
			temp_point.z = 0.0;
			real_points->points.push_back(temp_point);
		}
	}
	real_sub_flag = true;
}

void StaticLocalMap_callback(nav_msgs::OccupancyGrid msg){
	static_map = msg;
	size_t map_size = msg.data.size();

	static_points->clear();
	for(size_t i=0;i<map_size;i++){
		if(msg.data[i] != 0){
			pcl::PointXYZ temp_point;
			temp_point.x = (i % msg.info.width) * msg.info.resolution + msg.info.origin.position.x;
			temp_point.y = (int)(i / msg.info.width) * msg.info.resolution + msg.info.origin.position.y;
			temp_point.z = 0.0;
			static_points->points.push_back(temp_point);
		}
	}
	static_sub_flag = true;
}

void PointXYZ2PointCloud2(pcl::PointCloud<pcl::PointXYZ>::Ptr *input, sensor_msgs::PointCloud2 *output){
	pcl::PCLPointCloud2 pcl_pointcloud2;
	pcl::toPCLPointCloud2(**input, pcl_pointcloud2);
	pcl_conversions::fromPCL(pcl_pointcloud2, *output);
}


int main(int argc, char** argv){
	ros::init(argc, argv, "example12");

	ros::NodeHandle n;

	ros::Subscriber sub1 = n.subscribe("/local_map_real", 1, RealLocalMap_callback);
	ros::Subscriber sub2 = n.subscribe("/local_map_static", 1, StaticLocalMap_callback);
	ros::Subscriber sub3 = n.subscribe("/lcl", 1, lcl_callback);

	ros::Publisher vis_pub_real = n.advertise<sensor_msgs::PointCloud2>("/vis_real_points", 1);
	ros::Publisher vis_pub_static = n.advertise<sensor_msgs::PointCloud2>("/vis_static_points", 1);
	ros::Publisher vis_pub_output = n.advertise<sensor_msgs::PointCloud2>("/vis_output_points", 1);
	
	ros::Publisher pub_lcl_ndt = n.advertise<nav_msgs::Odometry>("/lcl_ndt", 10);

	clock_t start, end;

	ros::Rate loop_rate(20);
	
	while(ros::ok()){
		vis_real_points.header.stamp = ros::Time::now();	
		vis_static_points.header.stamp = ros::Time::now();	
		
		if(real_sub_flag && static_sub_flag && odom_sub_flag){
			PointXYZ2PointCloud2(&real_points, &vis_real_points);
			PointXYZ2PointCloud2(&static_points, &vis_static_points);

			cout<<"real_points.size() : "<<real_points->size()<<endl;
			cout<<"static_points.size() : "<<static_points->size()<<endl;
			vis_real_points.header.frame_id = "/velodyne";
			vis_static_points.header.frame_id = "/velodyne";
			vis_pub_real.publish(vis_real_points);
			vis_pub_static.publish(vis_static_points);
			
			pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
			approximate_voxel_filter.setLeafSize(0.3, 0.3, 0.3);
			approximate_voxel_filter.setInputCloud(real_points);
			// approximate_voxel_filter.setInputCloud(static_points);
			approximate_voxel_filter.filter(*filtered_points);
			cout<<"filtered_points.size() : "<<filtered_points->size()<<endl;
			
			pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_target_points (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter_target;
			approximate_voxel_filter_target.setLeafSize(0.3, 0.3, 0.3);
			approximate_voxel_filter_target.setInputCloud(static_points);
			// approximate_voxel_filter_target.setInputCloud(real_points);
			approximate_voxel_filter_target.filter(*filtered_target_points);
			cout<<"filtered_target_points.size() : "<<filtered_target_points->size()<<endl;

			pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
			ndt.setTransformationEpsilon(0.01);
			ndt.setStepSize(0.1);
			ndt.setResolution(1.0);
			ndt.setMaximumIterations(35);
			// ndt.setInputSource(static_points);
			ndt.setInputSource(filtered_points);
			// ndt.setInputTarget(real_points);
			ndt.setInputTarget(filtered_target_points);

			// Eigen::AngleAxisf init_rotation(odom.pose.pose.orientation.z - M_PI / 2.0, Eigen::Vector3f::UnitZ());
			Eigen::AngleAxisf init_rotation(0.0, Eigen::Vector3f::UnitZ());
			// Eigen::Translation3f init_translation(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
			Eigen::Translation3f init_translation(0.0, 0.0, 0.0);
			Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();
			
			// cout<<"NDT has converged : "<<ndt.hasConverged()<<endl;

			pcl::PointCloud<pcl::PointXYZ>::Ptr output_points (new pcl::PointCloud<pcl::PointXYZ>);
			
			start = clock();
			ndt.align(*output_points, init_guess);
			end = clock();

			PointXYZ2PointCloud2(&output_points, &vis_output_points);
			vis_output_points.header.frame_id = "/velodyne";
			vis_output_points.header.stamp = ros::Time::now();	


			cout<<"ndt.getFinalTransformation() : "<<endl<<ndt.getFinalTransformation()<<endl;
			cout<<"ndt.hasConverged() : "<<ndt.hasConverged()<<endl;
			cout<<"ndt.getFitnessScore() : "<<ndt.getFitnessScore()<<endl;
			printf("execution time : %f [sec]\n", (double)(end-start) / CLOCKS_PER_SEC);
			cout<<"ndt.getFinalNumIteration() : "<<ndt.getFinalNumIteration()<<endl;
			// cout<<"ndt.getFinalTransformation()(3,0) : "<<endl<<ndt.getFinalTransformation()(0,3)<<endl;
			// cout<<"ndt.getFinalTransformation()(3,1) : "<<endl<<ndt.getFinalTransformation()(1,3)<<endl;
				
			odom.pose.pose.position.x -= ndt.getFinalTransformation()(1, 3);
			odom.pose.pose.position.y -= ndt.getFinalTransformation()(0, 3);
			// odom.pose.pose.position.x += 1.0;
			// odom.pose.pose.position.y += 0.0;

			real_sub_flag = false;
			static_sub_flag = false;
			odom_sub_flag = false;
		}
		pub_lcl_ndt.publish(odom);
		vis_pub_output.publish(vis_output_points);


		// cout<<"real_map.header.frame_id : "<<real_map.header.frame_id<<endl;
		// cout<<"static_map.header.frame_id : "<<static_map.header.frame_id<<endl;
		// cout<<"vis_real_points.header.frame_id : "<<vis_real_points.header.frame_id<<endl;
		// cout<<"vis_static_points.header.frame_id : "<<vis_static_points.header.frame_id<<endl;
		// printf("1 loop hz : %f [hz]\n", 1.0 /((double)(end-start) / CLOCKS_PER_SEC));
		
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
