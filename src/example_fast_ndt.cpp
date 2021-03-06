#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "fast_pcl/registration/ndt.h"
// #include <pcl/registration/ndt.h>
#include "fast_pcl/filters/voxel_grid.h"
// #include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include <time.h>
using namespace std;

clock_t start, end_;

int
main (int argc, char** argv)
{
  // Loading first scan of room.
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  // if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/amsl/ros_catkin_ws/src/my_pcl_tutorial/test_velodyne.pcd", *target_cloud) == -1)
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/amsl/ros_catkin_ws/src/my_pcl_tutorial/room_scan1.pcd", *target_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " << target_cloud->size () << " data points from room_scan1.pcd" << std::endl;

  // Loading second scan of room from new perspective.
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  // if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/amsl/ros_catkin_ws/src/my_pcl_tutorial/test_velodyne2.pcd", *input_cloud) == -1)
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/amsl/ros_catkin_ws/src/my_pcl_tutorial/room_scan2.pcd", *input_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file room_scan2.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " << input_cloud->size () << " data points from room_scan2.pcd" << std::endl;

  // Filtering input scan to roughly 10% of original size to increase speed of registration.
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setLeafSize (0.1, 0.1, 0.1);
  voxel_filter.setInputCloud (input_cloud);
  voxel_filter.filter (*filtered_cloud);
  std::cout << "Filtered cloud contains " << filtered_cloud->size ()
			<< " data points from room_scan2.pcd" << std::endl;

  // Initializing Normal Distributions Transform (NDT).
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

  // Setting scale dependent NDT parameters
  // Setting minimum transformation difference for termination condition.
  ndt.setTransformationEpsilon (0.01);
  // Setting maximum step size for More-Thuente line search.
  ndt.setStepSize (0.1);
  //Setting Resolution of NDT grid structure (VoxelGridCovariance).
  ndt.setResolution (1.0);

  // Setting max number of registration iterations.
  ndt.setMaximumIterations (35);

  // Setting point cloud to be aligned.
  ndt.setInputSource (filtered_cloud);
  // ndt.setInputSource (input_cloud);
  // Setting point cloud to be aligned to.
  ndt.setInputTarget (target_cloud);

  // Set initial alignment estimate found using robot odometry.
  Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
  // Eigen::AngleAxisf init_rotation (0.0, Eigen::Vector3f::UnitZ ());
  // Eigen::AngleAxisf init_rotation (30.0 / 180.0 * M_PI, Eigen::Vector3f::UnitZ ());
  // Eigen::AngleAxisf r (M_PI, Eigen::Vector3f::UnitZ());
  // cout<<"AngleAxisf : init_rotation : "<<endl<<r.matrix()<<endl;
  Eigen::Translation3f init_translation (0.0, 0.0, 0.0);
  // Eigen::Translation3f init_translation (1.79387, 0.720047, 0.0);
  // Eigen::Translation3f init_translation (2.0, 2.0, 0.0);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

  cout<<"init_guess : "<<endl<<init_guess<<endl;
  // Calculating required rigid transform to align the input cloud to the target cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  
  start = clock();
  // ndt.align (*output_cloud, init_guess);
  ndt.omp_align (*output_cloud, init_guess);
  end_ = clock();
  std::cout<<"execution time : "<<(double)(end_-start) / CLOCKS_PER_SEC<<" [sec] "<<endl;;
  std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
            << " score: " << ndt.getFitnessScore () << std::endl;
  // cout<<"getoulierratio : "<<ndt.getOulierRatio()<<endl;
  cout<<"getFinalNumIteration() : "<<ndt.getFinalNumIteration()<<endl;
  cout<<"getfinaltransformation() : "<<endl<<ndt.getFinalTransformation()<<endl;

  // Transforming unfiltered, input cloud using found transform.
  pcl::transformPointCloud (*input_cloud, *output_cloud, ndt.getFinalTransformation());

  // Saving transformed input cloud.
  pcl::io::savePCDFileASCII ("/home/amsl/ros_catkin_ws/src/my_pcl_tutorial/room_scan2_transformed.pcd", *output_cloud);

  // Initializing point cloud visualizer
  boost::shared_ptr<pcl::visualization::PCLVisualizer>
  viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer_final->setBackgroundColor (0, 0, 0);

  // Coloring and visualizing target cloud (red).
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  target_color (target_cloud, 255, 0, 0);
  viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "target cloud");

  // Coloring and visualizing transformed input cloud (green).
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  output_color (output_cloud, 0, 255, 0);
  viewer_final->addPointCloud<pcl::PointXYZ> (output_cloud, output_color, "output cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "output cloud");

  // Starting visualizer
  viewer_final->addCoordinateSystem (1.0, "global");
  viewer_final->initCameraParameters ();

  // Wait until visualizer window is closed.
  while (!viewer_final->wasStopped ())
  {
    viewer_final->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

  return (0);
}
