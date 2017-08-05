#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>

using namespace std;

pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;

void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

	pcl_conversions::toPCL(*cloud_msg, *cloud);
	cout<<cloud<<endl;

}

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "test velodyne");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "test velodyne");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

int main (int argc, char** argv){
	// Initialize ROS
	ros::init (argc, argv, "example6");
	ros::NodeHandle n;
	
	// ros::Publisher pub=n.advertise<pcl::PointCloud<pcl::PointXYZ> >("output", 1);
	ros::Publisher pub=n.advertise<sensor_msgs::PointCloud2>("output", 1);
	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = n.subscribe ("velodyne_points", 1, cloud_callback);
	
	pcl::PCDWriter writer;
	
	ros::Rate loop_rate(10);
	int count = 0;
	while(ros::ok()){
		cout<<"asdaasdasdasdas"<<endl;
		ros::spinOnce();
		if(count==30){
		  writer.write ("test_velodyne.pcd", *cloud, 
				 Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

			break;
		}
		count ++;
		loop_rate.sleep();
	}


	return 0;
}
