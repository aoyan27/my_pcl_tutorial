#include <ros/ros.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "example_statistical_outlier_removal");


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PCDReader reader;
	reader.read ("/home/amsl/ros_catkin_ws/src/my_pcl_tutorial/sample_pcd/table_scene_lms400.pcd", *cloud);

	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << *cloud << std::endl;

	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.setNegative(false);
	sor.filter(*filtered_cloud);

	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *filtered_cloud << std::endl;

	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZRGB>("/home/amsl/ros_catkin_ws/src/my_pcl_tutorial/sample_pcd/table_scene_lms400_inliers.pcd", *filtered_cloud, false);

	sor.filter(*filtered_cloud);
	writer.write<pcl::PointXYZRGB>("/home/amsl/ros_catkin_ws/src/my_pcl_tutorial/sample_pcd/table_scene_lms400_outliers.pcd", *filtered_cloud, false);

	return 0;
}
