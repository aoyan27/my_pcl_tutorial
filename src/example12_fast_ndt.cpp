#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Odometry.h>

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

using namespace std;

nav_msgs::OccupancyGrid real_map;
nav_msgs::OccupancyGrid static_map;

pcl::PointCloud<pcl::PointXYZ>::Ptr real_points (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr static_points (new pcl::PointCloud<pcl::PointXYZ>);

sensor_msgs::PointCloud2 vis_real_points;
sensor_msgs::PointCloud2 vis_static_points;
sensor_msgs::PointCloud2 vis_output_points;

nav_msgs::Odometry odom;
nav_msgs::Odometry tiny;

boost::mutex real_;
boost::mutex static_;
boost::mutex odom_;
boost::mutex tiny_;


bool real_sub_flag = false;
// bool real_sub_flag = true;
bool static_sub_flag = false;
// bool static_sub_flag = true;
bool odom_sub_flag = false;
// bool odom_sub_flag = true;
bool tiny_sub_flag = false;
// bool tiny_sub_flag = true;

float dt = 1.0;
float velocity = 0.0;
float yaw = 0.0;
float pitch = 0.0;

float dyaw = 0.0;

const float dyaw_threshold = 0.10;

void amu_callback(catkin_ceres_msgs::AMU_data::ConstPtr msg){
	pitch = M_PI * msg->pitch / 180.0;
	dyaw = M_PI * msg->dyaw / 180.0;
	dyaw +=  0.0041196016 - 0.00025 + 0.000171428;
	// cout<<"msg->pitch : "<<msg->pitch<<endl;
	// cout<<"pitch : "<<pitch<<endl;
}

void odom_callback(nav_msgs::Odometry msg){
	boost::mutex::scoped_lock(tiny_);
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
	boost::mutex::scoped_lock(odom_);
	odom = msg;
	// cout<<"odom.pose.position.x : "<<odom.pose.pose.position.x<<endl;
	// cout<<"odom.pose.position.y : "<<odom.pose.pose.position.y<<endl;
	// cout<<"odom.pose.position.z : "<<odom.pose.pose.position.z<<endl;
	// cout<<"odom.pose.orientation.z : "<< odom.pose.pose.orientation.z<<endl;
	yaw = odom.pose.pose.orientation.z;
	odom_sub_flag = true;
}

void RealLocalMap_callback(nav_msgs::OccupancyGrid msg){
	boost::mutex::scoped_lock(real_);
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

	size_t real_points_size = real_points->points.size();
	// cout<<"resl_points_size : "<<real_points_size<<endl;
	if(real_points_size != 0){
		real_sub_flag = true;
	}
}

void StaticLocalMap_callback(nav_msgs::OccupancyGrid msg){
	boost::mutex::scoped_lock(static_);
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

	size_t static_points_size = static_points->points.size();
	// cout<<"static_points_size : "<<static_points_size<<endl;
	if(static_points_size != 0){
		static_sub_flag = true;
	}
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
	// ros::Subscriber sub3 = n.subscribe("/lcl", 1, lcl_callback);
	// ros::Subscriber sub3 = n.subscribe("/lcl4", 1, lcl_callback);
	// ros::Subscriber sub3 = n.subscribe("/ekf_lcl", 1, lcl_callback);
	// ros::Subscriber sub3 = n.subscribe("/ekf_ndt", 1, lcl_callback);
	ros::Subscriber sub3 = n.subscribe("/ekf_DgaussAndNDT", 1, lcl_callback);
	ros::Subscriber sub4 = n.subscribe("/tinypower/odom", 1, odom_callback);
	ros::Subscriber sub5 = n.subscribe("/AMU_data", 1, amu_callback);
	


	ros::Publisher vis_pub_real = n.advertise<sensor_msgs::PointCloud2>("/vis_real_points", 1);
	ros::Publisher vis_pub_static = n.advertise<sensor_msgs::PointCloud2>("/vis_static_points", 1);
	ros::Publisher vis_pub_output = n.advertise<sensor_msgs::PointCloud2>("/vis_output_points", 1);
	
	ros::Publisher pub_lcl_ndt = n.advertise<nav_msgs::Odometry>("/lcl_ndt", 10);
	ros::Publisher pub_vis_lcl_ndt = n.advertise<nav_msgs::Odometry>("/vis_lcl_ndt", 10);

	clock_t start, end;
	
	int count = 0;
	int count_ = 0;

	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	ndt.setTransformationEpsilon(0.01);
	ndt.setStepSize(0.1);
	ndt.setResolution(1.0);
	ndt.setMaximumIterations(30);

	ros::Rate loop_rate(1);
	
	nav_msgs::Odometry vis_odom;

	while(ros::ok()){
		// cout<<"NDT Start!!!!"<<endl;
		vis_real_points.header.stamp = ros::Time::now();	
		vis_static_points.header.stamp = ros::Time::now();	
		
		if(real_sub_flag && static_sub_flag && odom_sub_flag && tiny_sub_flag){
			PointXYZ2PointCloud2(&real_points, &vis_real_points);
			PointXYZ2PointCloud2(&static_points, &vis_static_points);

			// cout<<"real_points.size() : "<<real_points->size()<<endl;
			// cout<<"static_points.size() : "<<static_points->size()<<endl;
			vis_real_points.header.frame_id = "/velodyne";
			vis_static_points.header.frame_id = "/velodyne";
			vis_pub_real.publish(vis_real_points);
			vis_pub_static.publish(vis_static_points);
			
			pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
			voxel_filter.setLeafSize(0.3, 0.3, 0.3);
			voxel_filter.setInputCloud(real_points);
			// approximate_voxel_filter.setInputCloud(static_points);
			voxel_filter.filter(*filtered_points);
			// cout<<"filtered_points.size() : "<<filtered_points->size()<<endl;
			
			pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_target_points (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_target;
			voxel_filter_target.setLeafSize(0.3, 0.3, 0.3);
			voxel_filter_target.setInputCloud(static_points);
			// approximate_voxel_filter_target.setInputCloud(real_points);
			voxel_filter_target.filter(*filtered_target_points);
			// cout<<"filtered_target_points.size() : "<<filtered_target_points->size()<<endl;

			// ndt.setInputSource(static_points);
			ndt.setInputSource(filtered_points);
			// ndt.setInputTarget(real_points);
			ndt.setInputTarget(filtered_target_points);

			// Eigen::AngleAxisf init_rotation(odom.pose.pose.orientation.z - M_PI / 2.0, Eigen::Vector3f::UnitZ());
			Eigen::AngleAxisf init_rotation(0.0, Eigen::Vector3f::UnitZ());
			// Eigen::Translation3f init_translation(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
			Eigen::Translation3f init_translation(0.0, 0.0, 0.0);
			Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();
			
			// cout<<"init_guess : "<<init_guess<<endl;


			pcl::PointCloud<pcl::PointXYZ>::Ptr output_points (new pcl::PointCloud<pcl::PointXYZ>);
			// cout<<"output_points initialize!!!!!"<<endl;
			cout<<"dyaw : "<<dyaw<<endl;	
			cout<<"count_ : "<<count_<<endl;
			count_ ++;
			if(fabs(dyaw) < dyaw_threshold){
				cout<<"count : "<<count<<endl;
				count ++;
				start = clock();
				cout<<"start : "<<start<<endl;
				ndt.omp_align(*output_points, init_guess);
				cout<<"Just Finish NDT !!!"<<endl;
				end = clock();
				cout<<"end : "<<end<<endl;

				PointXYZ2PointCloud2(&output_points, &vis_output_points);
				vis_output_points.header.frame_id = "/velodyne";
				vis_output_points.header.stamp = ros::Time::now();	


				// cout<<"ndt.getFinalTransformation() : "<<endl<<ndt.getFinalTransformation()<<endl;
				// cout<<"ndt.hasConverged() : "<<ndt.hasConverged()<<endl;
				// cout<<"ndt.getFitnessScore() : "<<ndt.getFitnessScore()<<endl;
				printf("execution time : %f [sec]\n", (double)(end-start) / CLOCKS_PER_SEC);
				cout<<"ndt.getFinalNumIteration() : "<<ndt.getFinalNumIteration()<<endl;
				// cout<<"ndt.getFinalTransformation()(3,0) : "<<endl<<ndt.getFinalTransformation()(0,3)<<endl;
				// cout<<"ndt.getFinalTransformation()(3,1) : "<<endl<<ndt.getFinalTransformation()(1,3)<<endl;
					
				odom.pose.pose.position.x += ndt.getFinalTransformation()(1, 3);
				odom.pose.pose.position.y -= ndt.getFinalTransformation()(0, 3);
				// odom.pose.pose.position.x += 1.0;
				// odom.pose.pose.position.y += 0.0;
				vis_odom = odom;
			}
			// if(ndt.getFitnessScore() < 0.5){
			odom.pose.pose.position.x += velocity * cos(pitch) * cos(yaw) * dt;
 			odom.pose.pose.position.y += velocity * cos(pitch) * sin(yaw) * dt;
			odom.pose.pose.orientation.z -= dyaw * dt;
			pub_vis_lcl_ndt.publish(vis_odom);
			pub_lcl_ndt.publish(odom);
			vis_pub_output.publish(vis_output_points);
			// }

			real_sub_flag = false;
			static_sub_flag = false;
			odom_sub_flag = false;
			tiny_sub_flag = false;
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
