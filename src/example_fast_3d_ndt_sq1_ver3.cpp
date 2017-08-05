#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Odometry.h>
#include <catkin_ceres_msgs/AMU_data.h>

#include <sq1_msgs/OdometryBuffer.h>

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
#include <math.h>
#include <boost/thread.hpp>

#include "ray_casting.h"

using namespace std;

const float min_x = 0.0;
const float min_y = 0.0;

const float R = 0.3;
const float local_W = 20.0;
const float local_H = 20.0;

const float position_x = (min_x - local_W) / 2.0;
const float position_y = (min_y - local_H) / 2.0;

nav_msgs::OccupancyGrid real_map;
nav_msgs::OccupancyGrid static_map;

pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_data (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr local_map_points (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr velodyne_points_ (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr velodyne_points__ (new pcl::PointCloud<pcl::PointXYZ>);

std_msgs::Header header_;
std_msgs::Header ekf_header_;

sensor_msgs::PointCloud2 vis_output_points;

sensor_msgs::PointCloud2 vis_local_map_points;
sensor_msgs::PointCloud2 vis_velodyne_points;

nav_msgs::Odometry odom_;

sq1_msgs::OdometryBuffer odom_buffer_;

catkin_ceres_msgs::AMU_data amu_;

boost::mutex mutex_amu;
boost::mutex mutex_odom;
boost::mutex mutex_velodyne;


bool odom_sub_flag = false;
// bool odom_sub_flag = true;
bool velodyne_sub_flag = false;

float dt = 1.0;
float velocity = 0.0;
float yaw = 0.0;
float pitch = 0.0;

float dyaw = 0.0;

const float dyaw_threshold = 1.0;
// const float dyaw_threshold = 0.10;
// const float dyaw_threshold = 0.04;
// const float dyaw_threshold = 0.15;

float x_ = 0.0;
float y_ = 0.0;
float z_ = 0.0;
// const float init_yaw = (-90.0) / 180.0 * M_PI;
// const float init_yaw = (-7.2) / 180.0 * M_PI;
const float init_yaw = (0.0) / 180.0 * M_PI;

const float leaf_size = 0.3;
const float leaf_size_src = 1.0;
// const float leaf_size_src = 0.3;

int count_ = 0;
int index_ = 0;

void PointCloud2_to_PointXYZ(sensor_msgs::PointCloud2 *input, pcl::PointCloud<pcl::PointXYZ>::Ptr *output){
	pcl::PCLPointCloud2 pcl_cloud2;
	pcl_conversions::toPCL(*input, pcl_cloud2);
	pcl::fromPCLPointCloud2(pcl_cloud2, **output);
}

void PointXYZ2PointCloud2(pcl::PointCloud<pcl::PointXYZ>::Ptr *input, sensor_msgs::PointCloud2 *output){
	pcl::PCLPointCloud2 pcl_pointcloud2;
	pcl::toPCLPointCloud2(**input, pcl_pointcloud2);
	pcl_conversions::fromPCL(pcl_pointcloud2, *output);
}


int check_time(std_msgs::Header target_header, sq1_msgs::OdometryBuffer buffer){
	int index = 0;
	double min = fabs((buffer.times.data[0].sec + 1e-9*buffer.times.data[0].nsec) - (target_header.stamp.sec + 1e-9*target_header.stamp.nsec));
	double temp_min;
	for(size_t i=1; i<buffer.times.data.size();i++){
		temp_min = fabs((buffer.times.data[static_cast<int>(i)].sec + 1e-9*buffer.times.data[static_cast<int>(i)].nsec) - (target_header.stamp.sec + 1e-9*target_header.stamp.nsec));
		if(temp_min < min){
			min = temp_min;
			index = static_cast<int>(i);
		}
	}
	cout<<"min : "<<min<<endl;
	cout<<"index : "<<index<<endl;

	return index;
}


void lcl_callback(nav_msgs::Odometry msg){
	boost::mutex::scoped_lock(mutex_odom);
	odom_ = msg;
	ekf_header_ = msg.header;
	// cout<<"ekf_header_ : "<<ekf_header_<<endl;
	// cout<<"buffer_size : "<<odom_buffer_.times.data.size()<<endl;
	// cout<<"append!!!!!!!!!!!!!!"<<endl;
	odom_buffer_.times.data.push_back(ekf_header_.stamp);
	odom_buffer_.poses.positions.x.push_back(odom_.pose.pose.position.x);
	odom_buffer_.poses.positions.y.push_back(odom_.pose.pose.position.y);
	odom_buffer_.poses.positions.z.push_back(odom_.pose.pose.position.z);
	odom_buffer_.poses.orientations.x.push_back(odom_.pose.pose.orientation.x);
	odom_buffer_.poses.orientations.y.push_back(odom_.pose.pose.orientation.y);
	odom_buffer_.poses.orientations.z.push_back(odom_.pose.pose.orientation.z);
	odom_buffer_.poses.orientations.w.push_back(odom_.pose.pose.orientation.w);

	count_++;
	// cout<<odom_buffer_<<endl;
	// cout<<"odom_.pose.position.x : "<<odom_.pose.pose.position.x<<endl;
	// cout<<"odom_.pose.position.y : "<<odom_.pose.pose.position.y<<endl;
	// cout<<"odom_.pose.position.z : "<<odom_.pose.pose.position.z<<endl;
	// cout<<"odom_.pose.orientation.z : "<< odom_.pose.pose.orientation.z<<endl;
	// tf::Quaternion q_(odom_.pose.pose.orientation.x, odom_.pose.pose.orientation.y, odom_.pose.pose.orientation.z, odom_.pose.pose.orientation.w);
	// double roll_, pitch_, yaw_;
	// tf::Matrix3x3(q_).getRPY(roll_, pitch_, yaw_);
	// cout<<"yaw_ : "<<yaw_<<endl;
	// yaw = yaw_;
	odom_sub_flag = true;
}

void velodyne_callback(const sensor_msgs::PointCloud2 msg){
	if(odom_buffer_.times.data.size() > 0){
		boost::mutex::scoped_lock(mutex_velodyne);
		sensor_msgs::PointCloud2 temp = msg;
		header_ = msg.header;
		// cout<<"msg.header : "<<msg.header<<endl;
		// cout<<"header_ : "<<header_<<endl;
		// temp.header.stamp = ros::Time::now();
		index_ = check_time(header_, odom_buffer_);
		// cout<<"index_ : "<<index_<<endl;

		PointCloud2_to_PointXYZ(&temp, &velodyne_points__);
		
		velodyne_points_->points.clear();
		size_t velodyne_size = velodyne_points__->points.size();
		for(size_t i = 0; i < velodyne_size; i++){
			pcl::PointXYZ temp_point;
			temp_point.x = velodyne_points__->points[i].x; 
			temp_point.y = velodyne_points__->points[i].y;
			temp_point.z = velodyne_points__->points[i].z;
			if((-30.0 <= temp_point.x && temp_point.x <= 30.0) && (-30.0 <= temp_point.y && temp_point.y <= 30.0)){
				velodyne_points_->points.push_back(temp_point);
			}
		}
		
		x_ = odom_buffer_.poses.positions.x[index_];
		y_ = odom_buffer_.poses.positions.y[index_];
		z_ = odom_buffer_.poses.positions.z[index_];
		// cout<<"x_ : "<<x_<<endl;
		// cout<<"y_ : "<<y_<<endl;
		// cout<<"z_ : "<<z_<<endl;


		tf::Quaternion q_(odom_buffer_.poses.orientations.x[index_], odom_buffer_.poses.orientations.y[index_], odom_buffer_.poses.orientations.z[index_], odom_buffer_.poses.orientations.w[index_]);
		double roll_, pitch_, yaw_;
		tf::Matrix3x3(q_).getRPY(roll_, pitch_, yaw_);
		// cout<<"roll_ : "<<roll_<<endl;
		// cout<<"pitch_ : "<<pitch_<<endl;
		// cout<<"yaw_ : "<<yaw_<<endl;
		yaw = yaw_;
	
		if(count_ > 10000){
			// cout<<"elase!!!!!!!!!!!!!!!"<<endl;
			for(int i=0;i<=index_;i++){
				cout<<"i : "<<i<<endl;
				odom_buffer_.times.data.erase(odom_buffer_.times.data.begin());
				odom_buffer_.poses.positions.x.erase(odom_buffer_.poses.positions.x.begin());
				odom_buffer_.poses.positions.y.erase(odom_buffer_.poses.positions.y.begin());
				odom_buffer_.poses.positions.z.erase(odom_buffer_.poses.positions.z.begin());
				odom_buffer_.poses.orientations.x.erase(odom_buffer_.poses.orientations.x.begin());
				odom_buffer_.poses.orientations.y.erase(odom_buffer_.poses.orientations.y.begin());
				odom_buffer_.poses.orientations.z.erase(odom_buffer_.poses.orientations.z.begin());
				odom_buffer_.poses.orientations.w.erase(odom_buffer_.poses.orientations.w.begin());
			}
		}

		velodyne_sub_flag = true;
	}
}

void amu_callback(catkin_ceres_msgs::AMU_data msg){
	amu_ = msg;
}


int main(int argc, char** argv){
	ros::init(argc, argv, "example_fast_3d_ndt_sq1");

	ros::NodeHandle n;

	// string filename = "/home/amsl/map_tsukuba_all2.pcd";
	// string filename = "/home/amsl/obs_cloud_data_tsukuba.pcd";
	// string filename = "/home/amsl/obs_cloud_data_tsukuba.pcd";
	// string filename = "/home/amsl/obs_map.pcd";
	// string filename = "/home/amsl/obs_cloud_data_ikuta.pcd";
	// string filename = "/home/amsl/obs_cloud_data_ikuta_filtered.pcd";
	// string filename = "/home/amsl/obs_cloud_data_tsukuba_filtered.pcd";
	// string filename = "/home/amsl/dkan_map.pcd";
	// string filename = "/home/amsl/haneda_map_0.pcd";
	string filename = "/home/amsl/haneda_map_sq1.pcd";
	if(pcl::io::loadPCDFile (filename, *global_map_data) == -1){
		PCL_ERROR("Couldn't read file '%s'", filename.c_str());
		return -1;
	}

	// ros::Subscriber sub3 = n.subscribe("/lcl", 1, lcl_callback);
	// ros::Subscriber sub3 = n.subscribe("/lc2", 1, lcl_callback);
	// ros::Subscriber sub3 = n.subscribe("/lcl4", 1, lcl_callback);
	ros::Subscriber sub_init_pose = n.subscribe("/ekf_ndt", 1, lcl_callback);
	// ros::Subscriber sub6 = n.subscribe("/velodyne_points", 1, velodyne_callback);
	// ros::Subscriber sub_src = n.subscribe("/lidar3d/point_cloud/filtered", 1, velodyne_callback);
	ros::Subscriber sub_src = n.subscribe("/lidar3d/point_cloud/lcl", 1, velodyne_callback);
	ros::Subscriber sub_amu = n.subscribe("/imu_navio/strapdown", 1, amu_callback);
	


	// ros::Publisher vis_pub_velodyne_points = n.advertise<sensor_msgs::PointCloud2>("/vis_velodyne_poitns", 1);
	ros::Publisher vis_pub_velodyne_points = n.advertise<sensor_msgs::PointCloud2>("/lidar3d/point_cloud/vis", 1);
	ros::Publisher vis_pub_3d_local_map = n.advertise<sensor_msgs::PointCloud2>("/vis_3d_local_map", 1);
	ros::Publisher vis_pub_output = n.advertise<sensor_msgs::PointCloud2>("/vis_output_points", 1);

	ros::Publisher pub_lcl_ndt = n.advertise<nav_msgs::Odometry>("/lcl_ndt", 10);

	clock_t start, end;

	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	ndt.setTransformationEpsilon(0.01);
	ndt.setStepSize(0.1);
	// ndt.setResolution(1.0);
	ndt.setResolution(3.0);
	ndt.setMaximumIterations(35);

	ros::Rate loop_rate(20);

	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_target_points (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_target;
	voxel_filter_target.setLeafSize(leaf_size, leaf_size, leaf_size);
	voxel_filter_target.setInputCloud(global_map_data);
	voxel_filter_target.filter(*filtered_target_points);
	ndt.setInputTarget(filtered_target_points);
	
	Eigen::Matrix4f t(Eigen::Matrix4f::Identity());
	// cout<<"t : "<<t<<endl;
	
	double l_roll, l_pitch, l_yaw;

	while(ros::ok()){
		pcl::PointCloud<pcl::PointXYZ>::Ptr velodyne_points (new pcl::PointCloud<pcl::PointXYZ>);
		{
			boost::mutex::scoped_lock(mutex_velodyne);
			velodyne_points = velodyne_points_;
		}
		nav_msgs::Odometry odom;
		{
			boost::mutex::scoped_lock(mutex_odom);
			odom = odom_;
		}
		// cout<<"NDT Start!!!!"<<endl;
		
		// vis_velodyne_points.header.stamp = ros::Time::now();	
		// vis_local_map_points.header.stamp = ros::Time::now();	

		vis_velodyne_points.header = header_;	
		vis_local_map_points.header = header_;	
		vis_output_points.header = header_;	
		
		// cout<<velodyne_sub_flag<<odom_sub_flag<<endl;

		if(velodyne_sub_flag && odom_sub_flag){
			// cout<<"just do it !!!"<<endl;
			
			PointXYZ2PointCloud2(&velodyne_points, &vis_velodyne_points);
			PointXYZ2PointCloud2(&global_map_data, &vis_local_map_points);
			vis_velodyne_points.header.frame_id = "/centerlaser";
			vis_local_map_points.header.frame_id = "/centerlaser";
			vis_pub_velodyne_points.publish(vis_velodyne_points);
			vis_pub_3d_local_map.publish(vis_local_map_points);

			pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
			voxel_filter.setLeafSize(leaf_size_src, leaf_size_src, leaf_size_src);
			voxel_filter.setInputCloud(velodyne_points);
			voxel_filter.filter(*filtered_points);
		
			ndt.setInputSource(filtered_points);

			Eigen::AngleAxisf init_rotation(yaw + init_yaw, Eigen::Vector3f::UnitZ());
			// Eigen::Translation3f init_translation(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
			Eigen::Translation3f init_translation(x_, y_, z_);
			Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();
			
			// cout<<"init_guess : "<<init_guess<<endl;


			pcl::PointCloud<pcl::PointXYZ>::Ptr output_points (new pcl::PointCloud<pcl::PointXYZ>);
			// cout<<"output_points initialize!!!!!"<<endl;
			// cout<<endl;
			// cout<<"pitch : "<<pitch<<endl;	
			// cout<<"yaw : "<<yaw<<endl;	
			// cout<<"dyaw : "<<dyaw<<endl;	
			if(fabs(amu_.dyaw) < 0.3){
				start = clock();
				// cout<<"start : "<<start<<endl;
				ndt.omp_align(*output_points, init_guess);
				// cout<<"Just Finish NDT !!!"<<endl;
				end = clock();
				// cout<<"end : "<<end<<endl;
				PointXYZ2PointCloud2(&output_points, &vis_output_points);
				

				// cout<<"ndt.getFinalTransformation() : "<<endl<<ndt.getFinalTransformation()<<endl;
				// cout<<"ndt.hasConverged() : "<<ndt.hasConverged()<<endl;
				// cout<<"ndt.getFitnessScore() : "<<ndt.getFitnessScore()<<endl;
				printf("execution time : %f [sec]\n", (double)(end-start) / CLOCKS_PER_SEC);
				dt =  (double)(end-start) / CLOCKS_PER_SEC;
				// cout<<"ndt.getFinalNumIteration() : "<<ndt.getFinalNumIteration()<<endl;
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
				cout<<"t(2,3) : "<<t(2, 3)<<endl;
				cout<<"l_roll : "<<l_roll<<endl;
				cout<<"l_pitch : "<<l_pitch<<endl;
				cout<<"l_yaw : "<<l_yaw - init_yaw<<endl<<endl;

				// odom.pose.pose.position.x = ndt.getFinalTransformation()(0, 3);
				odom.pose.pose.position.x = t(0, 3);
				// odom.pose.pose.position.y = ndt.getFinalTransformation()(1, 3);
				odom.pose.pose.position.y = t(1, 3);

				odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(l_yaw);

				// cout<<"odom_ndt : "<<odom<<endl;

				// vis_output_points.header.frame_id = "/velodyne";
				// cout<<"vis_output_points.header : "<<vis_output_points.header<<endl;
				vis_output_points.header.frame_id = "/map";
				vis_pub_output.publish(vis_output_points);
			}
			pub_lcl_ndt.publish(odom);

			odom_sub_flag = false;
			velodyne_sub_flag = false;
		}

		
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
