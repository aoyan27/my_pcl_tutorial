#include <ros/ros.h>

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/visualization/cloud_viewer.h>

using namespace std;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "example_plane_segmentation2");
	ros::NodeHandle n;


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	// if(pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/amsl/ros_catkin_ws/src/my_pcl_tutorial/sample_pcd/table_scene_lms400.pcd", *cloud) == -1){
	if(pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/amsl/ros_catkin_ws/src/my_pcl_tutorial/sample_pcd/test_velodyne.pcd", *cloud) == -1){
		PCL_ERROR ("Couldn't read pcd file \n");
		return -1;
	}

	// Generate the data
	for (size_t i = 0; i < cloud->points.size (); ++i){
		cloud->points[i].r = 255;
		cloud->points[i].g = 255;
		cloud->points[i].b = 255;
	}

	cerr << "Point cloud data: " << cloud->points.size () << " points" << endl;

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.1);

	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

	if(inliers->indices.size() == 0){
		PCL_ERROR("Could not estimate a planar model for the given dataset.");
		return -1;
	}

	cerr << "Model coefficients: " << coefficients->values[0] << " " 
								   << coefficients->values[1] << " "
								   << coefficients->values[2] << " " 
								   << coefficients->values[3] << endl;

	cerr << "Model inliers: " << inliers->indices.size () << endl;
	for (size_t i = 0; i < inliers->indices.size (); ++i){
		// cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
												   // << cloud->points[inliers->indices[i]].y << " "
												   // << cloud->points[inliers->indices[i]].z << endl;
		cloud->points[inliers->indices[i]].r = 255;
		cloud->points[inliers->indices[i]].g = 0;
		cloud->points[inliers->indices[i]].b = 0;
	}

	pcl::visualization::CloudViewer viewer("Cloud Viewer");

	viewer.showCloud(cloud);

	while(!viewer.wasStopped()){
	
	}

	return 0;
}
