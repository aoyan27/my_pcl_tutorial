// ROS headers 
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

// PCL TRUNK headers 
#include <pcl/pcl_config.h>
#include <pcl/common/common.h>
#include <pcl/console/time.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>

using namespace std;

int main (int argc, char **argv) 
{
  cout<< PCL_VERSION_PRETTY <<endl; 
  // Call a PCL 1.8.0 feature ! (don't forget the include) 
  return 0; 
}


