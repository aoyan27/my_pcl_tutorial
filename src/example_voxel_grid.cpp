#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

using namespace std;

int main (int argc, char** argv){
  // Loading first scan of room.
  string filename = "/home/amsl/obs_map.pcd";
  string filename_out  = "/home/amsl/ros_catkin_ws/src/my_pcl_tutorial/obs_map_filtered.pcd";
  pcl::PointCloud<pcl::PointSurfel>::Ptr input_cloud (new pcl::PointCloud<pcl::PointSurfel>);
  if (pcl::io::loadPCDFile<pcl::PointSurfel> (filename, *input_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read %s !!!\n", filename.c_str());
    return (-1);
  }
  std::cout << "Loaded " << input_cloud->size () << " data points from "<< filename << std::endl;

  // Filtering input scan to roughly 10% of original size to increase speed of registration.
  pcl::PointCloud<pcl::PointSurfel>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointSurfel>);
  pcl::ApproximateVoxelGrid<pcl::PointSurfel> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize (0.1, 0.1, 0.1);
  approximate_voxel_filter.setInputCloud (input_cloud);
  approximate_voxel_filter.filter (*filtered_cloud);
  std::cout << "Filtered cloud contains " << filtered_cloud->size ()
			<< " data points from " << filename << std::endl;

  // size_t input_size = input_cloud->size();
  // for(size_t i=0;i < input_size;i++){
      // cout<<"points["<<i<<"].x : "<<input_cloud->points[i].x<<endl;
      // cout<<"points["<<i<<"].y : "<<input_cloud->points[i].y<<endl;
      // cout<<"points["<<i<<"].z : "<<input_cloud->points[i].z<<endl;
      // cout<<"points["<<i<<"].curvature : "<<input_cloud->points[i].curvature<<endl;

  // }
  // Saving transformed input cloud.
 cout<<"Saving filtered cloud to "<<filename_out<<endl;
 pcl::io::savePCDFileASCII (filename_out, *filtered_cloud);
 cout<<"Finish!!!!!!!!!!!!!!!"<<endl;
//  // Initializing point cloud visualizer
//  boost::shared_ptr<pcl::visualization::PCLVisualizer>
//  viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//  viewer_final->setBackgroundColor (0, 0, 0);
//
//  // Coloring and visualizing target cloud (red).
//  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointSurfel>
//  target_color (target_cloud, 255, 0, 0);
//  viewer_final->addPointCloud<pcl::PointSurfel> (target_cloud, target_color, "target cloud");
//  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
//                                                  1, "target cloud");
//
//  // Coloring and visualizing transformed input cloud (green).
//  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointSurfel>
//  output_color (output_cloud, 0, 255, 0);
//  viewer_final->addPointCloud<pcl::PointSurfel> (output_cloud, output_color, "output cloud");
//  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
//                                                  1, "output cloud");
//
//  // Starting visualizer
//  viewer_final->addCoordinateSystem (1.0, "global");
//  viewer_final->initCameraParameters ();
//
//  // Wait until visualizer window is closed.
//  while (!viewer_final->wasStopped ())
//  {
//    viewer_final->spinOnce (100);
//    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//  }

  return (0);
}
