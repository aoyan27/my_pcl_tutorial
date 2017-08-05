#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/voxel_grid.h>

using namespace std;

double min_height = 0.2;
double max_height = 1.6;

const float W = 1500;
const float H = 1500;

const float min_x = 0.0;
const float min_y = 0.0;

const float R = 0.1;

void remove_point(pcl::PointCloud<pcl::PointSurfel>::Ptr cloud, pcl::PointCloud<pcl::PointSurfel>::Ptr temp_cloud){
	cout<<"cloud->points.size() : "<<cloud->points.size()<<endl;
	
	for(size_t i=0;i<cloud->points.size();i++){
		if((-96.9 <= cloud->points[i].x && cloud->points[i].x <= 8.8) && (-28.7 <= cloud->points[i].y && cloud->points[i].y <= 68.9)){
			pcl::PointSurfel temp_point;

			temp_point.x = cloud->points[i].x;
			temp_point.y = cloud->points[i].y;
			temp_point.z = cloud->points[i].z;
			temp_point.r = cloud->points[i].r;
			temp_point.g = cloud->points[i].g;
			temp_point.b = cloud->points[i].b;
			temp_point.a = cloud->points[i].a;
			temp_point.radius = cloud->points[i].radius;
			temp_point.confidence = cloud->points[i].confidence;
			temp_point.curvature = cloud->points[i].curvature;
			
			temp_cloud->push_back(temp_point);
		}

	}
	cout<<"temp_cloud->points.size() : "<<temp_cloud->points.size()<<endl;

}

void remove_point_by_height(pcl::PointCloud<pcl::PointSurfel>::Ptr cloud, nav_msgs::OccupancyGrid *map, pcl::PointCloud<pcl::PointSurfel>::Ptr temp_cloud){
	vector<float> data(W/R*H/R*10, 100.0);
	

	cout<<"cloud->points.size() : "<<cloud->points.size()<<endl;
	for(size_t i = 0; i < cloud->points.size(); i++){
		int x = int((cloud->points[i].x - map->info.origin.position.x) / R);
		// cout<<"x : "<<x<<endl;
		int y = int((cloud->points[i].y - map->info.origin.position.y) / R);
		// cout<<"y : "<<y<<endl;
		int num = x + y * map->info.width;
		// cout<<"num : "<<num<<endl;
		
		if(cloud->points[i].z < data[num]){
			data[num] = cloud->points[i].z;
		}
	}
	
	//min-max
	for(size_t i = 0; i < cloud->points.size(); i++){
		int x = int((cloud->points[i].x - map->info.origin.position.x) / R);
		// cout<<"x : "<<x<<endl;
		int y = int((cloud->points[i].y - map->info.origin.position.y) / R);
		// cout<<"y : "<<y<<endl;
		int num = x + y * map->info.width;
		if(cloud->points[i].z <= (data[num] + max_height)){
			// cout<<"data["<<num<<"] : "<<data[num]<<endl;
			// cout<<"temp_cloud->points["<<i<<"].x : "<<temp_cloud->points[i].x<<endl;
			// cout<<"temp_cloud->points["<<i<<"].y : "<<temp_cloud->points[i].y<<endl;
			// cout<<"temp_cloud->points["<<i<<"].z : "<<temp_cloud->points[i].z<<endl;
			// cout<<endl;
			pcl::PointSurfel temp_point;

			temp_point.x = cloud->points[i].x;
			temp_point.y = cloud->points[i].y;
			temp_point.z = cloud->points[i].z;
			temp_point.r = cloud->points[i].r;
			temp_point.g = cloud->points[i].g;
			temp_point.b = cloud->points[i].b;
			temp_point.a = cloud->points[i].a;
			temp_point.radius = cloud->points[i].radius;
			temp_point.confidence = cloud->points[i].confidence;
			temp_point.curvature = cloud->points[i].curvature;

			// cout<<"temp_point.x : "<<temp_point.x<<endl;
			// cout<<"temp_point.y : "<<temp_point.y<<endl;
			// cout<<"temp_point.z : "<<temp_point.z<<endl;
			temp_cloud->push_back(temp_point);
		}
	}
	cout<<"temp_cloud->points.size() : "<<temp_cloud->points.size()<<endl;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "example_pcd");
	ros::NodeHandle n;
	
	nav_msgs::OccupancyGrid obstacle_map;
	obstacle_map.header.frame_id = "/map";
	obstacle_map.data.resize(int(W / R) * int(H / R));
	obstacle_map.info.width = int(W / R);
	obstacle_map.info.height = int(H / R);
	obstacle_map.info.resolution = R;
	obstacle_map.info.origin.position.x = (min_x - W) / 2.0;
	obstacle_map.info.origin.position.y = (min_y - H) / 2.0;

	pcl::PointCloud<pcl::PointSurfel>::Ptr cloud_data (new pcl::PointCloud<pcl::PointSurfel>);
	pcl::PointCloud<pcl::PointSurfel>::Ptr temp_cloud_data (new pcl::PointCloud<pcl::PointSurfel>);
	pcl::PointCloud<pcl::PointSurfel>::Ptr filtered_cloud_data (new pcl::PointCloud<pcl::PointSurfel>);

	string filename = "/home/amsl/merged_colored_map.pcd";
	if(pcl::io::loadPCDFile (filename, *cloud_data) == -1){
		PCL_ERROR("Couldn't read file '%s'", filename.c_str());
		return -1;
	}
	
	// remove_point_by_height(cloud_data, &obstacle_map, temp_cloud_data);
	remove_point(cloud_data, temp_cloud_data);

	// pcl::VoxelGrid<pcl::PointSurfel> approximate_voxel_filter;
	// approximate_voxel_filter.setLeafSize(0.1, 0.1, 0.1);
	// approximate_voxel_filter.setInputCloud(temp_cloud_data);
	// approximate_voxel_filter.filter(*filtered_cloud_data);
	// cout<<"Raw Cloud points : "<<temp_cloud_data->size()<<endl;
	// cout<<"Filtered cloud points : "<<filtered_cloud_data->size()<<endl;

	pcl::io::savePCDFile("data.pcd", *temp_cloud_data);
	cout<<"PCD File save!!!!!!"<<endl;

	return 0;
}
