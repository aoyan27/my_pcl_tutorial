#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Odometry.h>

#include <tf/tf.h>

#include <catkin_ceres_msgs/AMU_data.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <fast_pcl/registration/ndt.h>
#include <fast_pcl/filters/voxel_grid.h>

#include <iostream>
#include <math.h>
#include <time.h>
#include <stdio.h>

#include <boost/thread.hpp>

#include "ray_casting.h"

using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_points (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr velodyne_points (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr velodyne_points_resize_ (new pcl::PointCloud<pcl::PointXYZ>);

sensor_msgs::PointCloud2 vis_output_points;

sensor_msgs::PointCloud2 vis_global_map_points;
sensor_msgs::PointCloud2 vis_velodyne_points;

nav_msgs::Odometry odom;
nav_msgs::Odometry tiny;

boost::mutex mutex_amu;
boost::mutex mutex_odom;
boost::mutex mutex_tiny;
boost::mutex mutex_velodyne;

bool odom_sub_flag = false;
bool tiny_sub_flag = false;
bool velodyne_sub_flag = false;

float dyaw = 0.0;

const float dyaw_threshold = 1.0;
// const float dyaw_threshold = 0.10;
// const float dyaw_threshold = 0.04;
// const float dyaw_threshold = 0.15;

const float init_x = 0.0;
const float init_y = 0.0;
const float init_yaw = (0.0) / 180.0 * M_PI;

float last_angle;
int n = 0;

float expand_angle(float now_angle){
	float angle_ = now_angle;
	if((now_angle * last_angle) < 0.0){
		if((now_angle - last_angle) < -M_PI*1.8){
			n++;
		}
		else if((now_angle - last_angle) > M_PI*1.8){
			n--;
		}
	}
	// cout<<"now_angle : "<<now_angle<<endl;
	// cout<<"last_angle : "<<last_angle<<endl;
	// cout<<"n : "<<n<<endl;
	angle_ += 2.0 * n * M_PI;
	// cout<<"angle_ : "<<angle_<<endl;
	last_angle = now_angle;
	return angle_;
}

void amu_callback(catkin_ceres_msgs::AMU_data::ConstPtr msg){
	boost::mutex::scoped_lock(mutex_amu);
	// pitch = M_PI * msg->pitch / 180.0;
	dyaw = M_PI * msg->dyaw / 180.0;
	dyaw +=  0.0041196016 - 0.00025 + 0.000171428;
	// cout<<"msg->pitch : "<<msg->pitch<<endl;
	// cout<<"pitch : "<<pitch<<endl;
}

void odom_callback(nav_msgs::Odometry msg){
	boost::mutex::scoped_lock(mutex_tiny);
	tiny = msg;
	tiny.header.frame_id = "/map";
	tiny.child_frame_id = "/matching_base_link";
	// cout<<"tiny.pose.pose.position.x : "<<tiny.pose.pose.position.x<<endl;
	// cout<<"tiny.pose.pose.position.y : "<<tiny.pose.pose.position.y<<endl;
	// cout<<"tiny.pose.pose.position.z : "<<tiny.pose.pose.position.z<<endl;
	// cout<<"tiny.twist.linear : "<<endl<<tiny.twist.twist.linear<<endl;
	tiny_sub_flag = true;
}

void lcl_callback(nav_msgs::Odometry msg){
	boost::mutex::scoped_lock(mutex_odom);
	odom = msg;
	// cout<<"odom.pose.position.x : "<<odom.pose.pose.position.x<<endl;
	// cout<<"odom.pose.position.y : "<<odom.pose.pose.position.y<<endl;
	// cout<<"odom.pose.position.z : "<<odom.pose.pose.position.z<<endl;
	// cout<<"odom.pose.orientation.z : "<< odom.pose.pose.orientation.z<<endl;
	odom_sub_flag = true;
}

void velodyne_callback(const sensor_msgs::PointCloud2 msg){
	boost::mutex::scoped_lock(mutex_velodyne);
	pcl::fromROSMsg(msg, *velodyne_points);

	velodyne_points_resize_->points.clear();
		
	size_t velodyne_size = velodyne_points->points.size();
	for(size_t i = 0; i < velodyne_size; i++){
		pcl::PointXYZ temp_point;
		temp_point.x = velodyne_points->points[i].x; 
		temp_point.y = velodyne_points->points[i].y;
		temp_point.z = velodyne_points->points[i].z;
		if((-10.0 <= temp_point.x && temp_point.x <= 10.0) && (-10.0 <= temp_point.y && temp_point.y <= 10.0)){
			velodyne_points_resize_->points.push_back(temp_point);
		}
	}

	velodyne_sub_flag = true;
}


int main(int argc, char** argv){
	ros::init(argc, argv, "example_fast_3d_ndt_kari");

	ros::NodeHandle n;

	string filename = "/home/amsl/d_kan_indoor/map_3d_ds.pcd";
	if(pcl::io::loadPCDFile (filename, *global_map_points) == -1){
		PCL_ERROR("Couldn't read file '%s'", filename.c_str());
		return -1;
	}

	// ros::Subscriber sub3 = n.subscribe("/lcl", 1, lcl_callback);
	// ros::Subscriber sub3 = n.subscribe("/lcl3", 1, lcl_callback);
	// ros::Subscriber sub3 = n.subscribe("/lcl6", 1, lcl_callback);
	// ros::Subscriber sub3 = n.subscribe("/ekf_lcl", 1, lcl_callback);
	// ros::Subscriber sub3 = n.subscribe("/ekf_ndt", 1, lcl_callback);
	ros::Subscriber sub3 = n.subscribe("/ekf_DgaussAndNDT", 1, lcl_callback);
	ros::Subscriber sub4 = n.subscribe("/tinypower/odom", 1, odom_callback);
	// ros::Subscriber sub4 = n.subscribe("/t_frog/odom", 1, odom_callback);
	ros::Subscriber sub5 = n.subscribe("/AMU_data", 1, amu_callback);
	ros::Subscriber sub6 = n.subscribe("/velodyne_points", 1, velodyne_callback);
	// ros::Subscriber sub6 = n.subscribe("/lidar3d/point_cloud", 1, velodyne_callback);
	// ros::Subscriber sub6 = n.subscribe("/rm_ground2", 1, velodyne_callback);
	// ros::Subscriber sub6 = n.subscribe("/velodyne_obstacles", 1, velodyne_callback);
	


	ros::Publisher vis_pub_velodyne_points = n.advertise<sensor_msgs::PointCloud2>("/vis_velodyne_poitns", 1);
	// ros::Publisher vis_pub_velodyne_points = n.advertise<sensor_msgs::PointCloud2>("/lidar3d/point_cloud/vis", 1);
	ros::Publisher vis_pub_global_map_points = n.advertise<sensor_msgs::PointCloud2>("/vis_global_map_points", 1);
	ros::Publisher vis_pub_output_points = n.advertise<sensor_msgs::PointCloud2>("/vis_output_points", 1);
	
	ros::Publisher pub_lcl_ndt = n.advertise<nav_msgs::Odometry>("/lcl_ndt", 10);

	clock_t start, end;

	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	ndt.setTransformationEpsilon(0.01);
	ndt.setStepSize(0.1);
	ndt.setResolution(1.0);
	ndt.setMaximumIterations(30);

	ros::Rate loop_rate(10);

	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_target_points (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_target;
	voxel_filter_target.setLeafSize(0.3, 0.3, 0.3);
	voxel_filter_target.setInputCloud(global_map_points);
	voxel_filter_target.filter(*filtered_target_points);

	ndt.setInputTarget(filtered_target_points);
	
	Eigen::Matrix4f t(Eigen::Matrix4f::Identity());
	
	double l_roll, l_pitch, l_yaw;

	last_angle = init_yaw;

	while(ros::ok()){
		pcl::PointCloud<pcl::PointXYZ>::Ptr velodyne_points_resize (new pcl::PointCloud<pcl::PointXYZ>);
		{
			boost::mutex::scoped_lock(mutex_velodyne);
			velodyne_points_resize = velodyne_points_resize_;
		}
		cout<<"NDT Start!!!!"<<endl;
		
		vis_velodyne_points.header.stamp = ros::Time::now();	
		vis_global_map_points.header.stamp = ros::Time::now();	
		
		if(velodyne_sub_flag && odom_sub_flag && tiny_sub_flag){
			cout<<"just do it !!!"<<endl;
			
			pcl::toROSMsg(*velodyne_points_resize, vis_velodyne_points);
			pcl::toROSMsg(*global_map_points, vis_global_map_points);
			vis_velodyne_points.header.frame_id = "/velodyne";
			vis_global_map_points.header.frame_id = "/map";
			vis_pub_velodyne_points.publish(vis_velodyne_points);
			vis_pub_global_map_points.publish(vis_global_map_points);


			pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
			voxel_filter.setLeafSize(0.3, 0.3, 0.3);
			voxel_filter.setInputCloud(velodyne_points_resize);
			voxel_filter.filter(*filtered_points);
		
			ndt.setInputSource(filtered_points);

			Eigen::AngleAxisf init_rotation(odom.pose.pose.orientation.z + init_yaw, Eigen::Vector3f::UnitZ());
			Eigen::Translation3f init_translation(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
			Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();
			
			// cout<<"init_guess : "<<init_guess<<endl;


			pcl::PointCloud<pcl::PointXYZ>::Ptr output_points (new pcl::PointCloud<pcl::PointXYZ>);
			cout<<endl;
			cout<<"dyaw : "<<dyaw<<endl;	
			start = clock();
			cout<<"start : "<<start<<endl;
			ndt.omp_align(*output_points, init_guess);
			cout<<"Just Finish NDT !!!"<<endl;
			end = clock();
			cout<<"end : "<<end<<endl;

			pcl::toROSMsg(*output_points, vis_output_points);

			// cout<<"ndt.getFinalTransformation() : "<<endl<<ndt.getFinalTransformation()<<endl;
			// cout<<"ndt.hasConverged() : "<<ndt.hasConverged()<<endl;
			// cout<<"ndt.getFitnessScore() : "<<ndt.getFitnessScore()<<endl;
			printf("execution time : %f [sec]\n", (double)(end-start) / CLOCKS_PER_SEC);
			cout<<"ndt.getFinalNumIteration() : "<<ndt.getFinalNumIteration()<<endl;
			t = ndt.getFinalTransformation();
			// cout<<"ndt.getFinalTransformation() : "<<endl<<ndt.getFinalTransformation()<<endl;
			// cout<<"ndt.getFinalTransformation()(3,0) : "<<endl<<ndt.getFinalTransformation()(0,3)<<endl;
			// cout<<"ndt.getFinalTransformation()(3,1) : "<<endl<<ndt.getFinalTransformation()(1,3)<<endl;
			
			
			tf::Matrix3x3 mat_l;
			mat_l.setValue(static_cast<double>(t(0, 0)), static_cast<double>(t(0, 1)), static_cast<double>(t(0, 2)),
						   static_cast<double>(t(1, 0)), static_cast<double>(t(1, 1)), static_cast<double>(t(1, 2)),
						   static_cast<double>(t(2, 0)), static_cast<double>(t(2, 1)), static_cast<double>(t(2, 2)));
			
			mat_l.getRPY(l_roll, l_pitch, l_yaw, 1);

			cout<<"t(0,3) : "<<t(0, 3)<<endl;
			cout<<"t(1,3) : "<<t(1, 3)<<endl;
			cout<<"l_roll : "<<l_roll<<endl;
			cout<<"l_pitch : "<<l_pitch<<endl;
			cout<<"l_yaw : "<<l_yaw - init_yaw<<endl;

			// odom.pose.pose.position.x = ndt.getFinalTransformation()(0, 3);
			odom.pose.pose.position.x = t(0, 3);
			// odom.pose.pose.position.y = ndt.getFinalTransformation()(1, 3);
			odom.pose.pose.position.y = t(1, 3);

			odom.pose.pose.orientation.z = expand_angle(l_yaw - init_yaw);


			pub_lcl_ndt.publish(odom);
			// vis_output_points.header.frame_id = "/velodyne";
			vis_output_points.header.frame_id = "/map";
			vis_pub_output_points.publish(vis_output_points);
			// }

			odom_sub_flag = false;
			tiny_sub_flag = false;
			velodyne_sub_flag = false;
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
