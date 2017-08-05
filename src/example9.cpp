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

#include <nav_msgs/OccupancyGrid.h>


using namespace std;

const float W = 100;
const float H = 100;

const float min_x = 0.0;
const float min_y = 0.0;

const float R = 0.1;
// const float R = 1.0;

void expand_circle(int x0, int y0, int radius, nav_msgs::OccupancyGrid *map){
	int x = radius;
	int y = 0;
	int err = 0;
	
	cout<<"x0 : "<<x0<<endl;
	cout<<"y0 : "<<y0<<endl;
	cout<<"radius : "<<radius<<endl;

	while(x >= y){
		if((0 <= x && x < W/R) && (0 <= y && y < H/R)){
			cout<<"(x0+x)+(y0+y)*map->info.width : "<<(x0+x)+(y0+y)*map->info.width<<endl;
			map->data[(x0+x)+(y0+y)*map->info.width] = 100;
			cout<<"(x0+y)+(y0+x)*map->info.width : "<<(x0+y)+(y0+x)*map->info.width<<endl;
			map->data[(x0+y)+(y0+x)*map->info.width] = 100;
			cout<<"(x0-y)+(y0+x)*map->info.width : "<<(x0-y)+(y0+x)*map->info.width<<endl;
			map->data[(x0-y)+(y0+x)*map->info.width] = 100;
			cout<<"(x0-x)+(y0+y)*map->info.width : "<<(x0-x)+(y0+y)*map->info.width<<endl;
			map->data[(x0-x)+(y0+y)*map->info.width] = 100;
			cout<<"(x0-x)+(y0-y)*map->info.width : "<<(x0-x)+(y0-y)*map->info.width<<endl;
			map->data[(x0-x)+(y0-y)*map->info.width] = 100;
			cout<<"(x0-y)+(y0-x)*map->info.width : "<<(x0-y)+(y0-x)*map->info.width<<endl;
			map->data[(x0-y)+(y0-x)*map->info.width] = 100;
			cout<<"(x0+y)+(y0-x)*map->info.width : "<<(x0+y)+(y0-x)*map->info.width<<endl;
			map->data[(x0+y)+(y0-x)*map->info.width] = 100;
			cout<<"(x0+x)+(y0-y)*map->info.width : "<<(x0+x)+(y0-y)*map->info.width<<endl;
			map->data[(x0+x)+(y0-y)*map->info.width] = 100;

			y+=1;
			err += 1 + 2*y;
			if(2*(err-x)+1>0){
				x -= 1;
				err += 1 -2*x;
			}
		}
	}
	for(int i = 0; i < map->data.size(); i++){
		if(map->data[i] != 0){
			cout<<"i : "<<i<<endl;
		}
	}
}

int main (int argc, char** argv)
{
	ros::init (argc, argv, "example9");
	ros::NodeHandle n;
	
	ros::Publisher pub = n.advertise<nav_msgs::OccupancyGrid>("/test_map", 1);

	nav_msgs::OccupancyGrid test_map;
	test_map.header.frame_id = "/map";
	test_map.data.resize(long(W/R) * long(H/R));
	test_map.info.width = long(W/R);
	test_map.info.height = long(H/R);
	test_map.info.resolution = R;
	test_map.info.origin.position.x = (min_x-W)/2.0;
	test_map.info.origin.position.y = (min_y-H)/2.0;
	
	vector<int8_t>::iterator itr;
	for(itr = test_map.data.begin();itr != test_map.data.end(); itr++){
		*itr = 0;
	}
	
	expand_circle(int((W-min_x)/2.0/R), int((H-min_y)/2.0/R), int(1.0/R), &test_map);

	ros::Rate loop_rate(10);
	while(ros::ok()){
		pub.publish(test_map);
		loop_rate.sleep();
	}
	return 0;

}
