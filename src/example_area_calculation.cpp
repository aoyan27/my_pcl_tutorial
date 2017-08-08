#include <ros/ros.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/visualization/cloud_viewer.h>

using namespace std;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "example_extract_indices");
	ros::NodeHandle n;
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PCDReader reader;
	reader.read ("/home/amsl/ros_catkin_ws/src/my_pcl_tutorial/sample_pcd/table_scene_lms400_plane_1.pcd", *cloud);

	for(size_t i = 0; i < cloud->points.size(); i++){
		cloud->points[i].r = 255;
		cloud->points[i].g = 0;
		cloud->points[i].b = 0;
	}

	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.setNegative(false);
	sor.filter(*filtered_cloud);
	
	int max_x_index = 0;
	int max_y_index = 0;
	int min_x_index = 0;
	int min_y_index = 0;

	float tmp_max_x = filtered_cloud->points[0].x;
	float tmp_max_y = filtered_cloud->points[0].y;
	float tmp_min_x = filtered_cloud->points[0].x;
	float tmp_min_y = filtered_cloud->points[0].y;

	for(size_t i=1; i<filtered_cloud->points.size(); i++){
		if(filtered_cloud->points[i].x > tmp_max_x){
			tmp_max_x = filtered_cloud->points[i].x;
			max_x_index = i;
		}
		if(filtered_cloud->points[i].y > tmp_max_y){
			tmp_max_y = filtered_cloud->points[i].y;
			max_y_index = i;
		}
		if(tmp_min_x > filtered_cloud->points[i].x){
			tmp_min_x = filtered_cloud->points[i].x;
			min_x_index = i;
		}
		if(tmp_min_y > filtered_cloud->points[i].y){
			tmp_min_y = filtered_cloud->points[i].y;
			min_y_index = i;
		}
	}

	cout<<"tmp_max_x("<<max_x_index<<") : "<<tmp_max_x<<endl;
	cout<<"tmp_max_y("<<max_y_index<<") : "<<tmp_max_y<<endl;
	cout<<"tmp_min_x("<<min_x_index<<") : "<<tmp_min_x<<endl;
	cout<<"tmp_min_y("<<min_y_index<<") : "<<tmp_min_y<<endl;
	filtered_cloud->points[max_x_index].g = 255;
	filtered_cloud->points[max_y_index].g = 255;
	filtered_cloud->points[min_x_index].g = 255;
	filtered_cloud->points[min_y_index].g = 255;
	
	float X = tmp_max_x - tmp_min_x;
	float Y = tmp_max_y - tmp_min_y;
	float A = X * Y;
	cout<<"Area : "<<A<<endl;
	pcl::visualization::CloudViewer viewer("Cloud Viewer");

	// viewer.showCloud(cloud);
	viewer.showCloud(filtered_cloud);

	while(!viewer.wasStopped()){
	
	}


	return 0;
}
