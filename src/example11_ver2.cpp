#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Bool.h>

using namespace std;

nav_msgs::OccupancyGrid real_local_map;
nav_msgs::OccupancyGrid static_local_map;

std_msgs::Bool map_mode;

const float min_x = 0.0;
const float min_y = 0.0;

const float R = 0.1;

const float local_W = 20.0;
const float local_H = 20.0;

bool sub_real_flag = false;
bool sub_static_flag = false;

void map_mode_callback(std_msgs::Bool msg){
	map_mode.data = msg.data;
}

void RealLocalMap_callback(nav_msgs::OccupancyGrid msg){
	real_local_map = msg;
	// real_local_map.header.frame_id = "/obstacle_local_map";
	cout<<"real_local_map.header.frame_id : "<<real_local_map.header.frame_id<<endl;
	sub_real_flag = true;
}

void StaticLocalMap_callback(nav_msgs::OccupancyGrid msg){
	static_local_map = msg;
	// static_local_map.header.frame_id = "/obstacle_local_map";
	cout<<"static_local_map.header.frame_id : "<<static_local_map.header.frame_id<<endl;
	sub_static_flag = true;
}

float rotation_x(float x, float y, float theta){
	float x_r;
	// if(theta >= 0){
	x_r = cos(theta) * x - sin(theta) * y;
	// }
	// else{
		// x_r = cos(theta) * x - sin(theta) * y;
	// }
	return x_r;
}

float rotation_y(float x, float y, float theta){
	float y_r;
	// if(theta >= 0){
	y_r = sin(theta) * x + cos(theta) * y;
	// }
	// else{
		// y_r = -1 * sin(theta) * x + cos(theta) * y;
	// }
	return y_r;
}
void real2obstacle(nav_msgs::OccupancyGrid *map){
	geometry_msgs::Point real_point, obstacle_point;
	nav_msgs::OccupancyGrid out;
	out.header.frame_id = "/obstacle_local_map";
	out.header.stamp = ros::Time::now();
	out.data.resize(int(local_W / R) * int(local_H / R));
	out.info.width = int(local_W / R);
	out.info.height = int(local_H / R);
	out.info.resolution = R;
	out.info.origin.position.x = (min_x - local_W) / 2.0;
	out.info.origin.position.y = (min_y - local_H) / 2.0;

	for(int i=0; i < map->data.size(); i++){
		// real_point.x = (i % map->info.width) * R + map->info.origin.position.x;
		// real_point.y = int(i / map->info.width) * R + map->info.origin.position.y;
		// real_point.z = 0;
		real_point.x = (i % map->info.width);
		real_point.y = int(i / map->info.width);
		real_point.z = 0;
		// cout<<"real_point : "<<real_point<<endl;
		
		// obstacle_point.x = rotation_x(real_point.x, real_point.y, -90.0/180.0 * M_PI);
		// obstacle_point.y = rotation_y(real_point.x, real_point.y, -90.0/180.0 * M_PI);
		// obstacle_point.z = real_point.z;
		// obstacle_point.x = real_point.y;
		// obstacle_point.y = (out.info.width-1) - real_point.x;
		// obstacle_point.z = real_point.z;
		obstacle_point.x = real_point.x;
		obstacle_point.y = real_point.y;
		obstacle_point.z = real_point.z;
		// cout<<"obstacle_point : "<<obstacle_point<<endl;
		// out.info.origin.position.x = rotation_x(map->info.origin.position.x, map->info.origin.position.y, 90.0 / 180.0 * M_PI);
		// out.info.origin.position.y = rotation_y(map->info.origin.position.x, map->info.origin.position.y, 90.0 / 180.0 * M_PI);


		// int x = int((obstacle_point.x - out.info.origin.position.x) / R);
		// int y = int((obstacle_point.y - out.info.origin.position.y) / R);
		int x = int(obstacle_point.x);
		int y = int(obstacle_point.y);
		int num = x + y * out.info.width;
		// cout<<"num : "<<num<<endl;
		out.data[num] = map->data[i];
	}
	*map = out;

}

void static2obstacle(nav_msgs::OccupancyGrid *map){
	geometry_msgs::Point real_point, obstacle_point;
	nav_msgs::OccupancyGrid out;
	out.header.frame_id = "/obstacle_local_map";
	out.header.stamp = ros::Time::now();
	out.data.resize(int(local_W / R) * int(local_H / R));
	out.info.width = int(local_W / R);
	out.info.height = int(local_H / R);
	out.info.resolution = R;
	out.info.origin.position.x = (min_x - local_W) / 2.0;
	out.info.origin.position.y = (min_y - local_H) / 2.0;

	for(int i=0; i < map->data.size(); i++){
		// real_point.x = (i % map->info.width) * R + map->info.origin.position.x;
		// real_point.y = int(i / map->info.width) * R + map->info.origin.position.y;
		// real_point.z = 0;
		real_point.x = (i % map->info.width);
		real_point.y = int(i / map->info.width);
		real_point.z = 0;
		// cout<<"real_point : "<<real_point<<endl;
		
		// obstacle_point.x = rotation_x(real_point.x, real_point.y, -90.0/180.0 * M_PI);
		// obstacle_point.y = rotation_y(real_point.x, real_point.y, -90.0/180.0 * M_PI);
		// obstacle_point.z = real_point.z;
		// obstacle_point.x = (out.info.height-1) - real_point.x;
		// obstacle_point.y = real_point.y;
		// obstacle_point.z = real_point.z;
		obstacle_point.x = real_point.x;
		obstacle_point.y = real_point.y;
		obstacle_point.z = real_point.z;
		// cout<<"obstacle_point : "<<obstacle_point<<endl;
		// out.info.origin.position.x = rotation_x(map->info.origin.position.x, map->info.origin.position.y, 90.0 / 180.0 * M_PI);
		// out.info.origin.position.y = rotation_y(map->info.origin.position.x, map->info.origin.position.y, 90.0 / 180.0 * M_PI);


		// int x = int((obstacle_point.x - out.info.origin.position.x) / R);
		// int y = int((obstacle_point.y - out.info.origin.position.y) / R);
		int x = int(obstacle_point.x);
		int y = int(obstacle_point.y);
		int num = x + y * out.info.width;
		// cout<<"num : "<<num<<endl;
		out.data[num] = map->data[i];
	}
	*map = out;

}

void create_local_map(nav_msgs::OccupancyGrid real_local, nav_msgs::OccupancyGrid static_local, nav_msgs::OccupancyGrid *map){
	vector<int8_t>::iterator mit;
	for(mit = map->data.begin(); mit != map->data.end(); mit++){
		*mit = 0;
	}
	cout<<"map_mode."<<map_mode<<endl;
	for(int i=0; i<map->data.size(); i++){
		// cout<<"map->data["<<i<<"]() : "<<map->data[i]<<endl;
		// cout<<"real_local.data["<<i<<"] : "<<real_local.data[i]<<endl;
		// cout<<"static_local.data["<<i<<"] : "<<static_local.data[i]<<endl;
		if(map_mode.data){
			if(real_local.data[i] != 0){
				map->data[i] = 100;
			}
		}
		else{
			if(static_local.data[i] != 0 || real_local.data[i] != 0){
				map->data[i] = 100;
				// cout<<"map->data["<<i<<"] : "<<map->data[i]<<endl;
			}
		}
	}
	map_mode.data = false;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "example11");

	ros::NodeHandle n;

	ros::Subscriber sub1 = n.subscribe("/local_map_real/expand", 1, RealLocalMap_callback);
	ros::Subscriber sub2 = n.subscribe("/local_map_static/expand", 1, StaticLocalMap_callback);
	
	ros::Subscriber sub3 = n.subscribe("/arm1_start", 1, map_mode_callback);

	ros::Publisher pub = n.advertise<nav_msgs::OccupancyGrid>("/local_map", 1);

	nav_msgs::OccupancyGrid local_map;
	// local_map.header.frame_id = "/obstacle_local_map";
	local_map.header.frame_id = "/velodyne";
	
	local_map.data.resize(int(local_W / R) * int(local_H / R));
	local_map.info.width = int(local_W / R);
	local_map.info.height = int(local_H / R);
	local_map.info.resolution = R;
	local_map.info.origin.position.x = (min_x - local_W) / 2.0;
	local_map.info.origin.position.y = (min_y - local_H) / 2.0;
	

	ros::Rate loop_rate(10);

	while(ros::ok()){
		if(sub_real_flag && sub_static_flag){
			real2obstacle(&real_local_map);
			static2obstacle(&static_local_map);
			create_local_map(real_local_map, static_local_map, &local_map);
			sub_real_flag = false;
			sub_static_flag = false;
		}

		pub.publish(local_map);

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
