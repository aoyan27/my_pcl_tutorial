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

const float min_x = 0.0;
const float min_y = 0.0;

const float R = 0.3;
const float local_W = 20.0;
const float local_H = 20.0;

const float position_x = (min_x - local_W) / 2.0;
const float position_y = (min_y - local_H) / 2.0;

nav_msgs::OccupancyGrid real_map;
nav_msgs::OccupancyGrid static_map;

pcl::PointCloud<pcl::PointXYZ>::Ptr real_points (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr static_points (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr real_points_rc (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr static_points_rc (new pcl::PointCloud<pcl::PointXYZ>);


pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_data (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr local_map_points (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr velodyne_points_ (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr velodyne_points__ (new pcl::PointCloud<pcl::PointXYZ>);

sensor_msgs::PointCloud2 vis_real_points;
sensor_msgs::PointCloud2 vis_static_points;
sensor_msgs::PointCloud2 vis_output_points;

sensor_msgs::PointCloud2 vis_real_points_rc;
sensor_msgs::PointCloud2 vis_static_points_rc;

sensor_msgs::PointCloud2 vis_local_map_points;
sensor_msgs::PointCloud2 vis_velodyne_points;

nav_msgs::Odometry odom;
nav_msgs::Odometry tiny;

boost::mutex mutex_amu;
boost::mutex mutex_odom;
boost::mutex mutex_tiny;
boost::mutex mutex_velodyne;


bool real_sub_flag = false;
// bool real_sub_flag = true;
bool static_sub_flag = false;
// bool static_sub_flag = true;
bool odom_sub_flag = false;
// bool odom_sub_flag = true;
bool tiny_sub_flag = false;
// bool tiny_sub_flag = true;
bool velodyne_sub_flag = false;

float dt = 1.0;
float velocity = 0.0;
float yaw = 0.0;
float pitch = 0.0;

float dyaw = 0.0;

float last_angle;
int n = 0;

const float dyaw_threshold = 1.0;
// const float dyaw_threshold = 0.10;
// const float dyaw_threshold = 0.04;
// const float dyaw_threshold = 0.15;

const float init_x = 0.0;
const float init_y = 0.0;
// const float init_yaw = (-90.0) / 180.0 * M_PI;
// const float init_yaw = (-7.2) / 180.0 * M_PI;
const float init_yaw = (-90.0) / 180.0 * M_PI;

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

void amu_callback(catkin_ceres_msgs::AMU_data::ConstPtr msg){
	boost::mutex::scoped_lock(mutex_amu);
	pitch = M_PI * msg->pitch / 180.0;
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
	velocity = tiny.twist.twist.linear.x ;
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
	// tf::Quaternion q_(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
	double yaw_;
	yaw_ = odom.pose.pose.orientation.z;
	// tf::Matrix3x3(q_).getRPY(roll_, pitch_, yaw_);
	// cout<<"yaw_ : "<<yaw_<<endl;
	yaw = yaw_;
	odom_sub_flag = true;
}

void velodyne_callback(const sensor_msgs::PointCloud2 msg){
	boost::mutex::scoped_lock(mutex_velodyne);
	sensor_msgs::PointCloud2 temp = msg;
	temp.header.stamp = ros::Time::now();
	PointCloud2_to_PointXYZ(&temp, &velodyne_points__);
	
	velodyne_points_->points.clear();
	size_t velodyne_size = velodyne_points__->points.size();
	for(size_t i = 0; i < velodyne_size; i++){
		pcl::PointXYZ temp_point;
		temp_point.x = velodyne_points__->points[i].x; 
		temp_point.y = velodyne_points__->points[i].y;
		temp_point.z = velodyne_points__->points[i].z;
		if((-10.0 <= temp_point.x && temp_point.x <= 10.0) && (-10.0 <= temp_point.y && temp_point.y <= 10.0)){
			velodyne_points_->points.push_back(temp_point);
		}
	}

	// pcl::PointCloud<pcl::PointXYZ>::Ptr out_point_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	// Eigen::Matrix4d plus_pi;
	// Eigen::Affine plus_pi;
	// plus_pi = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ());
	// Eigen::AngleAxisd( radian, Vector3d( 0, 1, 0 ) );
	// pcl::transformPointCloud( velodyne_points_, out_point_cloud, plus_pi );
	// velodyne_points_ = out_point_cloud;

	// cout<<"velodyne_points_->points.size()"<<velodyne_points_->points.size()<<endl;
	velodyne_sub_flag = true;
}

void create_3d_local_map(float x, float y, float theta, pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output){
	// cout<<"input->points.size() : "<<input->points.size()<<endl;
	
	output->points.clear();

	size_t input_size = input->points.size();
	
	cout <<"x(3d) : "<<x<<endl; 
	cout <<"y(3d) : "<<y<<endl; 
	for(size_t i = 0; i < input_size; i++){
		pcl::PointXYZ temp_point;
		temp_point.x = rotation_x(input->points[i].x - x, input->points[i].y - y, -1*theta);
		temp_point.y = rotation_y(input->points[i].x - x, input->points[i].y - y, -1*theta);
		temp_point.z = input->points[i].z;
		
		if((-10.0 <= temp_point.x && temp_point.x <= 10.0) && (-10.0 <= temp_point.y && temp_point.y <= 10.0)){
			output->points.push_back(temp_point);
		}
	}
	// cout<<"output->points.size() : "<<output->points.size()<<endl;
	
}

void calc_z_offset(pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr velodyne_cloud){
	vector<float> data_map(local_W / R * local_H / R, 100.0);
	vector<float> data_velodyne(local_W / R * local_H / R, 100.0);
	vector<float> data_z_offset(local_W / R * local_H / R, 0.0);
	size_t map_cloud_size = map_cloud->points.size();
	size_t velodyne_cloud_size = velodyne_cloud->points.size();
	size_t data_size = data_map.size();	

	for(size_t i = 0; i < map_cloud_size; i++){
		int x = int((map_cloud->points[i].x - position_x ) / R);
		// cout<<"x : "<<x<<endl;
		int y = int((map_cloud->points[i].y - position_y) / R);
		// cout<<"y : "<<y<<endl;
		int num = x + y * int(local_W / R);
		// cout<<"num : "<<num<<endl;
		
		if(map_cloud->points[i].z < data_map[num]){
			data_map[num] = map_cloud->points[i].z;
		}
		// cout<<"data_map["<<num<<"] : "<<data_map[num]<<endl;
	}

	for(size_t i = 0; i < velodyne_cloud_size; i++){
		int x = int((velodyne_cloud->points[i].x - position_x ) / R);
		// cout<<"x : "<<x<<endl;
		int y = int((velodyne_cloud->points[i].y - position_y) / R);
		// cout<<"y : "<<y<<endl;
		int num = x + y * int(local_W / R);
		// cout<<"num : "<<num<<endl;
		
		if(velodyne_cloud->points[i].z < data_velodyne[num]){
			data_velodyne[num] = velodyne_cloud->points[i].z;
		}
		// cout<<"data_velodyne["<<num<<"] : "<<data_velodyne[num]<<endl;
	}
	
	for(size_t i=0; i<data_map.size();i++){
		if(data_map[i] == 100.0){
			data_map[i] = 0.0;
		}
		if(data_velodyne[i] == 100.0){
			data_velodyne[i] = 0.0;
		}
	}
	
	for(size_t i=0; i < data_size; i++){
		data_z_offset[i] = data_map[i] - data_velodyne[i];
	}

	for(size_t i = 0; i<map_cloud_size; i++){
		int x = int((map_cloud->points[i].x - position_x ) / R);
		// cout<<"x : "<<x<<endl;
		int y = int((map_cloud->points[i].y - position_y) / R);
		// cout<<"y : "<<y<<endl;
		int num = x + y * int(local_W / R);
		// cout<<"num : "<<num<<endl;
		// cout<<"data_z_offset["<<num<<"] : "<<data_z_offset[num]<<endl;
		map_cloud->points[i].z -= data_z_offset[num];
	}

}

int main(int argc, char** argv){
	ros::init(argc, argv, "example_fast_3d_ndt_sq1");

	ros::NodeHandle n;

	// string filename = "/home/amsl/velodyne_bin/d_center_left/0deg.pcd";
	// string filename = "/home/amsl/velodyne_bin/d_center_right/0deg.pcd";
	// string filename = "/home/amsl/velodyne_bin/d_chair_left/0deg.pcd";
	// string filename = "/home/amsl/velodyne_bin/d_chair_right/0deg.pcd";
	// string filename = "/home/amsl/velodyne_bin/a_kan_left/0deg.pcd";
	// string filename = "/home/amsl/velodyne_bin/a_kan_right/0deg.pcd";
	// string filename = "/home/amsl/velodyne_bin/nougaku_left/0deg.pcd";
	// string filename = "/home/amsl/velodyne_bin/nougaku_right/0deg.pcd";
	// string filename = "/home/amsl/jrm0521/a_kan_left/1/0deg.pcd";
	// string filename = "/home/amsl/jrm0521/a_kan_left/2/0deg.pcd";
	// string filename = "/home/amsl/jrm0521/a_kan_left/3/0deg.pcd";
	// string filename = "/home/amsl/jrm0521/a_kan_left/4/0deg.pcd";
	// string filename = "/home/amsl/jrm0521/a_kan_left/5/0deg.pcd";
	// string filename = "/home/amsl/jrm0521/nougaku_left/1/0deg.pcd";
	string filename = "/home/amsl/jrm0521/d_left/1/0deg.pcd";
	// string filename = "/home/amsl/jrm0521/d_left/2/0deg.pcd";
	// string filename = "/home/amsl/jrm0521/d_left/3/0deg.pcd";
	// string filename = "/home/amsl/jrm0521/d_left/4/0deg.pcd";
	// string filename = "/home/amsl/jrm0521/d_left/5/0deg.pcd";

	if(pcl::io::loadPCDFile (filename, *global_map_data) == -1){
		PCL_ERROR("Couldn't read file '%s'", filename.c_str());
		return -1;
	}

	// ros::Subscriber sub3 = n.subscribe("/lcl", 1, lcl_callback);
	// ros::Subscriber sub3 = n.subscribe("/lcl4", 1, lcl_callback);
	// ros::Subscriber sub3 = n.subscribe("/lcl6", 1, lcl_callback);
	// ros::Subscriber sub3 = n.subscribe("/ekf_lcl", 1, lcl_callback);
	// ros::Subscriber sub3 = n.subscribe("/ekf_ndt", 1, lcl_callback);
	ros::Subscriber sub3 = n.subscribe("/ekf_DgaussAndNDT", 1, lcl_callback);
	ros::Subscriber sub4 = n.subscribe("/tinypower/odom", 1, odom_callback);
	// ros::Subscriber sub4 = n.subscribe("/t_frog/odom", 1, odom_callback);
	ros::Subscriber sub5 = n.subscribe("/AMU_data", 1, amu_callback);
	ros::Subscriber sub6 = n.subscribe("/velodyne_points", 1, velodyne_callback);
	// ros::Subscriber sub6 = n.subscribe("/lidar3d/point_cloud", 1, velodyne_callback);
	// ros::Subscriber sub6 = n.subscribe("/lidar3d/point_cloud/lcl", 1, velodyne_callback);
	// ros::Subscriber sub6 = n.subscribe("/rm_ground2", 1, velodyne_callback);
	// ros::Subscriber sub6 = n.subscribe("/velodyne_obstacles", 1, velodyne_callback);
	


	ros::Publisher vis_pub_velodyne_points = n.advertise<sensor_msgs::PointCloud2>("/vis_velodyne_poitns", 1);
	// ros::Publisher vis_pub_velodyne_points = n.advertise<sensor_msgs::PointCloud2>("/lidar3d/point_cloud/vis", 1);
	ros::Publisher vis_pub_3d_local_map = n.advertise<sensor_msgs::PointCloud2>("/vis_3d_local_map", 1);
	ros::Publisher vis_pub_output = n.advertise<sensor_msgs::PointCloud2>("/vis_output_points", 1);

	// ros::Publisher vis_pub_real_rc = n.advertise<sensor_msgs::PointCloud2>("/vis_real_points/rc", 1);
	// ros::Publisher vis_pub_static_rc = n.advertise<sensor_msgs::PointCloud2>("/vis_static_points/rc", 1);
	
	ros::Publisher pub_lcl_ndt = n.advertise<nav_msgs::Odometry>("/lcl_ndt", 10);
	ros::Publisher pub_vis_lcl_ndt = n.advertise<nav_msgs::Odometry>("/vis_lcl_ndt", 10);

	clock_t start, end;
	
	int count = 0;
	int count_ = 0;

	// float l = 0.0;
	// float x = 0.0;
	// float y = 0.0;

	// float fai = 0.0;
	// float theta = 0.0;
	
	// float odom_x = init_x;
	// float odom_y = init_y;
	// float odom_yaw = init_yaw;

	last_angle = init_yaw;

	// pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points_ (new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;
	// voxel_filter_.setLeafSize(0.2, 0.2, 0.2);
	// voxel_filter.setInputCloud(real_points);
	// voxel_filter_.setInputCloud(global_map_data);
	// approximate_voxel_filter.setInputCloud(static_points);
	// voxel_filter_.filter(*filtered_points_);
	// cout<<"input_points_.size() : "<<global_map_data->size()<<endl;
	// cout<<"filtered_points_.size() : "<<filtered_points_->size()<<endl;

	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	ndt.setTransformationEpsilon(0.01);
	ndt.setStepSize(0.1);
	ndt.setResolution(1.0);
	ndt.setMaximumIterations(30);

	ros::Rate loop_rate(10);
	
	nav_msgs::Odometry vis_odom;

	// Eigen::AngleAxisf init_rotation(odom.pose.pose.orientation.z - M_PI / 2.0, Eigen::Vector3f::UnitZ());
	// Eigen::AngleAxisf init_rotation(0.0, Eigen::Vector3f::UnitZ());
	// Eigen::Translation3f init_translation(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
	// Eigen::Translation3f init_translation(0.0, 0.0, 0.0);
	// Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();
	
	// cout<<"init_guess : "<<init_guess<<endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_target_points (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_target;
	voxel_filter_target.setLeafSize(0.3, 0.3, 0.3);
	// voxel_filter_target.setInputCloud(static_points);
	// voxel_filter_target.setInputCloud(local_map_points);
	voxel_filter_target.setInputCloud(global_map_data);
	// approximate_voxel_filter_target.setInputCloud(real_points);
	voxel_filter_target.filter(*filtered_target_points);
	// cout<<"filtered_target_points.size() : "<<filtered_target_points->size()<<endl;
	ndt.setInputTarget(filtered_target_points);
	
	Eigen::Matrix4f t(Eigen::Matrix4f::Identity());
	// cout<<"t : "<<t<<endl;
	
	double l_roll, l_pitch, l_yaw;

	cout<<"NDT Start!!!!"<<endl;
	while(ros::ok()){
		pcl::PointCloud<pcl::PointXYZ>::Ptr velodyne_points (new pcl::PointCloud<pcl::PointXYZ>);
		{
			boost::mutex::scoped_lock(mutex_velodyne);
			velodyne_points = velodyne_points_;
		}
		
		// vis_real_points.header.stamp = ros::Time::now();	
		// vis_real_points_rc.header.stamp = ros::Time::now();	

		// vis_static_points.header.stamp = ros::Time::now();	
		// vis_static_points_rc.header.stamp = ros::Time::now();	

		// vis_output_points.header.stamp = ros::Time::now();	
		
		vis_velodyne_points.header.stamp = ros::Time::now();	
		vis_local_map_points.header.stamp = ros::Time::now();	
		
		// cout<<velodyne_sub_flag<<odom_sub_flag<<tiny_sub_flag<<endl;

		if(velodyne_sub_flag && odom_sub_flag && tiny_sub_flag){
			cout<<"just do it !!!"<<endl;
			// PointXYZ2PointCloud2(&real_points, &vis_real_points);
			// PointXYZ2PointCloud2(&static_points, &vis_static_points);
			// PointXYZ2PointCloud2(&real_points_rc, &vis_real_points_rc);
			// PointXYZ2PointCloud2(&static_points_rc, &vis_static_points_rc);
			// vis_real_points.header.frame_id = "/velodyne";
			// vis_real_points_rc.header.frame_id = "/velodyne";
			// vis_static_points.header.frame_id = "/velodyne";
			// vis_static_points_rc.header.frame_id = "/velodyne";

			// cout<<"real_points.size() : "<<real_points->size()<<endl;
			// cout<<"static_points.size() : "<<static_points->size()<<endl;
			// vis_pub_real.publish(vis_real_points);
			// vis_pub_real_rc.publish(vis_real_points_rc);
			// vis_pub_static.publish(vis_static_points);
			// vis_pub_static_rc.publish(vis_static_points_rc);
			
			PointXYZ2PointCloud2(&velodyne_points, &vis_velodyne_points);
			// PointXYZ2PointCloud2(&local_map_points, &vis_local_map_points);
			PointXYZ2PointCloud2(&global_map_data, &vis_local_map_points);
			vis_velodyne_points.header.frame_id = "/velodyne";
			vis_local_map_points.header.frame_id = "/map";
			vis_pub_velodyne_points.publish(vis_velodyne_points);
			vis_pub_3d_local_map.publish(vis_local_map_points);
			
			// odom_x = odom.pose.pose.position.x + init_x;
			// odom_y = odom.pose.pose.position.y + init_y;
			// odom_yaw = odom.pose.pose.orientation.z + init_yaw;
			// create_3d_local_map(odom_x, odom_y, odom_yaw, global_map_data, local_map_points);
			// create_3d_local_map(odom_x, odom_y, odom_yaw, filtered_points_, local_map_points);

			pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
			voxel_filter.setLeafSize(0.3, 0.3, 0.3);
			// voxel_filter.setInputCloud(real_points);
			voxel_filter.setInputCloud(velodyne_points);
			// approximate_voxel_filter.setInputCloud(static_points);
			voxel_filter.filter(*filtered_points);
			// cout<<"filtered_points.size() : "<<filtered_points->size()<<endl;
			
			
			// calc_z_offset(filtered_target_points, filtered_points);

			// PointXYZ2PointCloud2(&filtered_points, &vis_velodyne_points);
			// PointXYZ2PointCloud2(&filtered_target_points, &vis_local_map_points);
			// vis_velodyne_points.header.frame_id = "/velodyne";
			// vis_local_map_points.header.frame_id = "/velodyne";
			// vis_pub_velodyne_points.publish(vis_velodyne_points);
			// vis_pub_3d_local_map.publish(vis_local_map_points);
		
			// ndt.setInputSource(static_points);
			ndt.setInputSource(filtered_points);
			// ndt.setInputSource(real_points_rc);
			// ndt.setInputTarget(real_points);
			// ndt.setInputTarget(static_points_rc);

			// Eigen::AngleAxisf init_rotation(odom.pose.pose.orientation.z - M_PI / 2.0, Eigen::Vector3f::UnitZ());
			// Eigen::AngleAxisf init_rotation(0.0, Eigen::Vector3f::UnitZ());
			Eigen::AngleAxisf init_rotation(yaw + init_yaw, Eigen::Vector3f::UnitZ());
			// Eigen::Translation3f init_translation(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
			// Eigen::Translation3f init_translation(0.0, 0.0, 0.0);
			Eigen::Translation3f init_translation(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
			Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();
			
			// cout<<"init_guess : "<<init_guess<<endl;


			pcl::PointCloud<pcl::PointXYZ>::Ptr output_points (new pcl::PointCloud<pcl::PointXYZ>);
			// cout<<"output_points initialize!!!!!"<<endl;
			cout<<endl;
			// cout<<"pitch : "<<pitch<<endl;	
			cout<<"yaw : "<<yaw<<endl;	
			cout<<"dyaw : "<<dyaw<<endl;	
			cout<<"count_ : "<<count_<<endl;
			count_ ++;
			cout<<"count : "<<count<<endl;
			count ++;
			// cout<<"filtered_points.size() : "<<filtered_points->points.size()<<endl;
			// cout<<"filtered_target_points.size() : "<<filtered_target_points->points.size()<<endl;
			// cout<<"init_guess : "<<endl<<init_guess<<endl;
			start = clock();
			cout<<"start : "<<start<<endl;
			ndt.omp_align(*output_points, init_guess);
			cout<<"Just Finish NDT !!!"<<endl;
			end = clock();
			cout<<"end : "<<end<<endl;
			PointXYZ2PointCloud2(&output_points, &vis_output_points);
			

			// cout<<"ndt.getFinalTransformation() : "<<endl<<ndt.getFinalTransformation()<<endl;
			// cout<<"ndt.hasConverged() : "<<ndt.hasConverged()<<endl;
			// cout<<"ndt.getFitnessScore() : "<<ndt.getFitnessScore()<<endl;
			printf("execution time : %f [sec]\n", (double)(end-start) / CLOCKS_PER_SEC);
			dt =  (double)(end-start) / CLOCKS_PER_SEC;
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
			cout<<"t(2,3) : "<<t(2, 3)<<endl;
			cout<<"l_roll : "<<l_roll<<endl;
			cout<<"l_pitch : "<<l_pitch<<endl;
			cout<<"l_yaw(kari) : "<<l_yaw<<endl;
			float expand_yaw = expand_angle(l_yaw - init_yaw);
			cout<<"l_yaw : "<<expand_yaw<<endl;

			// odom.pose.pose.position.x = ndt.getFinalTransformation()(0, 3);
			odom.pose.pose.position.x = t(0, 3);
			// odom.pose.pose.position.y = ndt.getFinalTransformation()(1, 3);
			odom.pose.pose.position.y = t(1, 3);

			// odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(l_yaw);
			odom.pose.pose.orientation.z = expand_yaw;
			cout<<"Robot_yaw : "<<expand_yaw - 0.5 * M_PI<<", Robot_yaw(deg) : "<<(expand_yaw - 0.5 * M_PI) / M_PI * 180.0<<endl;
			// cout<<"odom_ndt : "<<odom<<endl;

			vis_odom = odom;
			// if(ndt.getFitnessScore() < 0.5){
			pub_vis_lcl_ndt.publish(vis_odom);
			pub_lcl_ndt.publish(odom);
			// vis_output_points.header.frame_id = "/velodyne";
			vis_output_points.header.frame_id = "/map";
			vis_pub_output.publish(vis_output_points);
			// }

			odom_sub_flag = false;
			tiny_sub_flag = false;
			velodyne_sub_flag = false;
		}


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
