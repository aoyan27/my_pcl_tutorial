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


// void 
// cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
// {
  // ... do data processing

  // sensor_msgs::PointCloud2 output;
  // Publish the data
  // pub.publish (output);

// }

// void 
// cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
// {
  // sensor_msgs::PointCloud2 cloud_filtered;

  // Perform the actual filtering
  // pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
  // sor.setInputCloud (cloud);
  // sor.setLeafSize (0.01, 0.01, 0.01);
  // sor.filter (cloud_filtered);

  // Publish the data
  // pub.publish (cloud_filtered);
// }
using namespace std;
int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle n;
  
  // ros::Publisher pub=n.advertise<pcl::PointCloud<pcl::PointXYZ> >("output", 1);
  ros::Publisher pub=n.advertise<sensor_msgs::PointCloud2>("output", 1);
  // Create a ROS subscriber for the input point cloud
  // ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read ("/home/amsl/ros_catkin_ws/src/my_pcl_tutorial/table_scene_lms400.pcd", *cloud); // Remember to download the file first!

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ").";

  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")."<<std::endl;

  pcl::PCDWriter writer;
  writer.write ("table_scene_lms400_downsampled.pcd", *cloud_filtered, 
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

  sensor_msgs::PointCloud2 cloud2;

  pcl_conversions::fromPCL(*cloud_filtered, cloud2);

  // cout<<cloud2<<endl;
  cloud2.header.frame_id = "/world";

  cout<<cloud2.header<<endl;

  ros::Rate loop_rate(10);
  while(ros::ok()){
	  cout<<"ADASDASD!"<<endl;
	  cloud2.header.stamp = ros::Time::now();
	  pub.publish(cloud2);
	  loop_rate.sleep();
  }
  return 0;
}
