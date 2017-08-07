#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <iostream>
#include <vector>
#include <ctime>

using namespace std;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "example_kdtree");
	ros::NodeHandle n;

	srand (time (NULL));

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	// Generate pointcloud data
	cloud->width = 1000;
	cloud->height = 1;
	cloud->points.resize (cloud->width * cloud->height);

	for (size_t i = 0; i < cloud->points.size (); ++i){
		cloud->points[i].x = 1024.0f * rand () / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024.0f * rand () / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024.0f * rand () / (RAND_MAX + 1.0f);
	}

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

	kdtree.setInputCloud (cloud);

	pcl::PointXYZ searchPoint;

	searchPoint.x = 1024.0f * rand () / (RAND_MAX + 1.0f);
	searchPoint.y = 1024.0f * rand () / (RAND_MAX + 1.0f);
	searchPoint.z = 1024.0f * rand () / (RAND_MAX + 1.0f);

	// K nearest neighbor search

	int K = 10;

	vector<int> pointIdxNKNSearch(K);
	vector<float> pointNKNSquaredDistance(K);

	cout << "K nearest neighbor search at (" << searchPoint.x 
			  << " " << searchPoint.y 
			  << " " << searchPoint.z
			  << ") with K=" << K << endl;

	if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
		for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i){
			cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x 
					  << " " << cloud->points[ pointIdxNKNSearch[i] ].y 
					  << " " << cloud->points[ pointIdxNKNSearch[i] ].z 
					  << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << endl;
		}
	}

	// Neighbors within radius search

	vector<int> pointIdxRadiusSearch;
	vector<float> pointRadiusSquaredDistance;

	float radius = 256.0f * rand () / (RAND_MAX + 1.0f);

	cout << "Neighbors within radius search at (" << searchPoint.x 
			  << " " << searchPoint.y 
			  << " " << searchPoint.z
			  << ") with radius=" << radius << endl;


	if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
		for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i){
  			cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x 
					  << " " << cloud->points[ pointIdxRadiusSearch[i] ].y 
					  << " " << cloud->points[ pointIdxRadiusSearch[i] ].z 
					  << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << endl;
		}
	}


	return 0;
}
