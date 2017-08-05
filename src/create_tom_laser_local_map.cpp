#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include <geometry_msgs/Point32.h>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <boost/thread.hpp>
#include <tf/transform_broadcaster.h>

#define Width 20
#define Length 20
#define Resolution 0.1

#define INV_Resolution 1.0/Resolution
#define HALF_WIDTH Width/2
#define HALF_LENGTH Length/2

#define MAP_WIDTH Width/Resolution
#define MAP_LENGTH Length/Resolution
#define MAP_SIZE MAP_WIDTH*MAP_LENGTH

#define _FREE 0
#define _UNEXPLORE 50
#define _LETHAL 100

#define CELL_FILTER 0

using namespace std;
using namespace Eigen;

sensor_msgs::PointCloud obs_220_, obs_221_, obs_222_, curvature_obs_220_, curvature_obs_221_, curvature_obs_222_;

boost::mutex mutex_220, mutex_221, mutex_222, mutex_curvature_220, mutex_curvature_221, mutex_curvature_222;

const float init_x = 0.0;
const float init_y = 0.28;
const float init_z = -0.4;
const float init_yaw = 90.0 / 180.0 * M_PI;

enum cost{FREE=0, LETHAL=100};


class MapIndex{
private:
	MapIndex();
public:
	MapIndex(int _i, int _j):i(_i),j(_j){	}
	MapIndex(const MapIndex& id):i(id.i),j(id.j){	}
	
	int i,j;
};
bool operator==(const MapIndex& lhs, const MapIndex& rhs){
	return ((lhs.i==rhs.i) && (lhs.j==rhs.j));
}

bool operator<(const MapIndex& lhs, const MapIndex& rhs){
	return ((1000*lhs.i+lhs.j) < (1000*rhs.i+rhs.j));
}



class ExpandMap{
private:

	list<MapIndex> expanded_circle;
	
	ExpandMap(const ExpandMap&);
	
public:
	ExpandMap(float _radius, float resolution);
	
	void expandObstacle(const nav_msgs::OccupancyGrid& map_in);
	void getGridCells(nav_msgs::GridCells& cells);
	
	nav_msgs::OccupancyGrid local_map;
};

// midpoint circle algorithm
ExpandMap::ExpandMap(float _radius, float resolution){
	int radius=round(_radius/resolution);

	int f=1-radius;
	int ddF_x=1;
	int ddF_y=-2*radius;
	int x=0;
	int y=radius;
	
	expanded_circle.push_back(MapIndex(0,radius));
	expanded_circle.push_back(MapIndex(0,-radius));
	expanded_circle.push_back(MapIndex(radius,0));
	expanded_circle.push_back(MapIndex(-radius,0));
	
	// draw circle line on grid map
	while(x<y){
		if(f>=0){
			y--;
			ddF_y+=2;
			f+=ddF_y;
		}
	
		x++;
		ddF_x+=2;
		f+=ddF_x;
		
		expanded_circle.push_back(MapIndex(x , y));
		expanded_circle.push_back(MapIndex(-x, y));
		expanded_circle.push_back(MapIndex(x ,-y));
		expanded_circle.push_back(MapIndex(-x,-y));
		if(x!=y){
		expanded_circle.push_back(MapIndex( y, x));
		expanded_circle.push_back(MapIndex(-y, x));
		expanded_circle.push_back(MapIndex( y,-x));
		expanded_circle.push_back(MapIndex(-y,-x));
		}
	}
	//cout<<"hello"<<endl;
	
	/* 20151103 */
	/*// fill the grids in the circle
	// bresenham's line algorithm
	int n=expanded_circle.size();
	list<MapIndex>::iterator itr=expanded_circle.begin();
	//for(itr=expanded_circle.begin(); itr!=expanded_circle.end(); itr++){
	for(int i=0; i<n; i+=2){
		int x1=itr->i;
		int y1=itr->j;
		itr++;
		int x2=itr->i;
		int y2=itr->j;
		itr++;
		bool steep= abs(y2-y1)>abs(x2-x1);
	
		if(steep){
			swap(x1,y1);
			swap(x2,y2);
		}
		if(x1>x2){
			swap(x1,x2);
			swap(y1,y2);		
		}
	
		int deltax=x2-x1;
		int deltay=abs(y2-y1);	
		float error=0;
		float deltaerr;
		if(deltax==0){
			deltaerr=10000;
		}else{
			deltaerr=deltay/deltax;	
		}
		int ystep;
		int yt=y1;
		if(y1<y2){
			ystep=1;
		}else{
			ystep=-1;
		}
		for(int xt=x1; xt<=x2; xt++){
			//cout<<xt<<":"<<x1<<","<<x2<<endl;
			if(steep){
				expanded_circle.push_back(MapIndex(yt,xt));
			}else{
				expanded_circle.push_back(MapIndex(xt,yt));
			}
		
			error+=deltaerr;
		
			if(error>=0.5){
				yt+=ystep;
				error-=1;
			}	
		}	
	}
	*/


	// delete several overlap grids
	expanded_circle.sort();
	expanded_circle.unique();
	
	//list<MapIndex>::iterator itr;
	/*
	for(itr=expanded_circle.begin(); itr!=expanded_circle.end(); itr++){
		cout<<itr->i<<"\t"<<itr->j<<endl;
	}*/
}

void ExpandMap::expandObstacle(const nav_msgs::OccupancyGrid& map_in){
	local_map=map_in;
	
	vector<int8_t>::iterator itr;
	for(itr=local_map.data.begin(); itr!=local_map.data.end(); itr++){
		*itr=FREE;
	}
	
	
	for(int xi=0; xi<(int)map_in.info.height; xi++){
		for(int yi=0; yi<(int)map_in.info.width; yi++){
			// if the cell is LETHAL
			if(map_in.data[xi+map_in.info.width*yi]!=FREE){
				// expand the LETHAL cells with respect to the circle radius
				list<MapIndex>::iterator litr;
				for(litr=expanded_circle.begin(); litr!=expanded_circle.end(); litr++){
					int x=xi+litr->i, y=yi+litr->j;
					if(x>=0 && x<(int)local_map.info.height && 	y>=0 && y<(int)local_map.info.width
						&& map_in.data[xi+map_in.info.width*yi]>local_map.data[x+map_in.info.width*y]){
						local_map.data[x+map_in.info.width*y]=map_in.data[xi+map_in.info.width*yi];
					}
				}
			}
		}
	}
}

//change(2011/09/30)
void ExpandMap::getGridCells(nav_msgs::GridCells& cells){
	cells.cells.clear();
	
	cells.header.frame_id=local_map.header.frame_id;
	cells.header.stamp=local_map.header.stamp;
	cells.cell_width=local_map.info.resolution;
	cells.cell_height=local_map.info.resolution;
	
	float _map_angle = tf::getYaw(local_map.info.origin.orientation);
	float map_angle = _map_angle - M_PI;
	
	
	for(int xi=0; xi<(int)local_map.info.height; xi++){
		for(int yi=0; yi<(int)local_map.info.width; yi++){
			if(local_map.data[xi+local_map.info.width*yi]!=FREE){
				float x_conv=cos(map_angle)*xi-sin(map_angle)*yi;
				float y_conv=sin(map_angle)*xi+cos(map_angle)*yi;
				
				float x=local_map.info.origin.position.x-x_conv*local_map.info.resolution;
				float y=local_map.info.origin.position.y-y_conv*local_map.info.resolution;//*/

				geometry_msgs::Point pt;
				pt.x=x;	pt.y=y;	pt.z=0;
				cells.cells.push_back(pt);
			}
		}
	}
}

float rotation_x(float x, float y, float yaw){
	return x * cos(yaw) - y * sin(yaw);
}

float rotation_y(float x, float y, float yaw){
	return x * sin(yaw) + y * cos(yaw);
}

void obs_220_callback(const sensor_msgs::PointCloud2 msg){
	boost::mutex::scoped_lock(mutex_220);
	sensor_msgs::PointCloud2 pc2;
	sensor_msgs::PointCloud pc;
	pc2 = msg;
	sensor_msgs::convertPointCloud2ToPointCloud(pc2, pc);
	obs_220_.points.clear();
	// cout<<"pc.points.size() : "<<pc.points.size()<<endl;
	for(size_t i=0; i<pc.points.size();i++){
		// cout<<"i : "<<i<<endl;
		geometry_msgs::Point32 temp_point;
		//temp_point.x = rotation_x(pc.points[i].x, pc.points[i].y, init_yaw) + init_x;
		//temp_point.y = rotation_y(pc.points[i].x, pc.points[i].y, init_yaw) + init_y;
		temp_point.x = pc.points[i].x;
		temp_point.y = pc.points[i].y, init_yaw;
		temp_point.z = pc.points[i].z + init_z;
		obs_220_.points.push_back(temp_point);
	}
	// cout<<"obs_220_.points.size() : "<<obs_220_.points.size()<<endl;
}

void obs_221_callback(const sensor_msgs::PointCloud2 msg){
	boost::mutex::scoped_lock(mutex_221);
	sensor_msgs::PointCloud2 pc2;
	sensor_msgs::PointCloud pc;
	pc2 = msg;
	sensor_msgs::convertPointCloud2ToPointCloud(pc2, pc);
	obs_221_.points.clear();
	// cout<<"pc.points.size() : "<<pc.points.size()<<endl;
	for(size_t i=0; i<pc.points.size();i++){
		// cout<<"i : "<<i<<endl;
		geometry_msgs::Point32 temp_point;
		//temp_point.x = rotation_x(pc.points[i].x, pc.points[i].y, init_yaw) + init_x;
		//temp_point.y = rotation_y(pc.points[i].x, pc.points[i].y, init_yaw) + init_y;
		temp_point.x = pc.points[i].x;
		temp_point.y = pc.points[i].y, init_yaw;
		temp_point.z = pc.points[i].z + init_z;
		obs_221_.points.push_back(temp_point);
	}
	// cout<<"obs_221_.points.size() : "<<obs_220_.points.size()<<endl;
}

void obs_222_callback(const sensor_msgs::PointCloud2 msg){
	boost::mutex::scoped_lock(mutex_222);
	sensor_msgs::PointCloud2 pc2;
	sensor_msgs::PointCloud pc;
	pc2 = msg;
	sensor_msgs::convertPointCloud2ToPointCloud(pc2, pc);
	obs_222_.points.clear();
	// cout<<"pc.points.size() : "<<pc.points.size()<<endl;
	for(size_t i=0; i<pc.points.size();i++){
		// cout<<"i : "<<i<<endl;
		geometry_msgs::Point32 temp_point;
		temp_point.x = rotation_x(pc.points[i].x, pc.points[i].y, init_yaw) + init_x;
		//temp_point.y = rotation_y(pc.points[i].x, pc.points[i].y, init_yaw) + init_y;
		//temp_point.z = pc.points[i].z + init_z;
		temp_point.x = pc.points[i].x;
		temp_point.y = pc.points[i].y, init_yaw;
		obs_222_.points.push_back(temp_point);
	}
	// cout<<"obs_222_.points.size() : "<<obs_222_.points.size()<<endl;
}

void curvature_obs_220_callback(const sensor_msgs::PointCloud2 msg){
	boost::mutex::scoped_lock(mutex_curvature_220);
	sensor_msgs::PointCloud2 pc2;
	sensor_msgs::PointCloud pc;
	pc2 = msg;
	sensor_msgs::convertPointCloud2ToPointCloud(pc2, pc);
	curvature_obs_220_.points.clear();
	// cout<<"pc.points.size() : "<<pc.points.size()<<endl;
	for(size_t i=0; i<pc.points.size();i++){
		// cout<<"i : "<<i<<endl;
		geometry_msgs::Point32 temp_point;
		temp_point.x = rotation_x(pc.points[i].x, pc.points[i].y, init_yaw) + init_x;
		temp_point.y = rotation_y(pc.points[i].x, pc.points[i].y, init_yaw) + init_y;
		temp_point.z = pc.points[i].z + init_z;
		curvature_obs_220_.points.push_back(temp_point);
	}
	// cout<<"obs_220_.points.size() : "<<obs_220_.points.size()<<endl;
}

void curvature_obs_221_callback(const sensor_msgs::PointCloud2 msg){
	boost::mutex::scoped_lock(mutex_curvature_221);
	sensor_msgs::PointCloud2 pc2;
	sensor_msgs::PointCloud pc;
	pc2 = msg;
	sensor_msgs::convertPointCloud2ToPointCloud(pc2, pc);
	curvature_obs_221_.points.clear();
	// cout<<"pc.points.size() : "<<pc.points.size()<<endl;
	for(size_t i=0; i<pc.points.size();i++){
		// cout<<"i : "<<i<<endl;
		geometry_msgs::Point32 temp_point;
		temp_point.x = rotation_x(pc.points[i].x, pc.points[i].y, init_yaw) + init_x;
		temp_point.y = rotation_y(pc.points[i].x, pc.points[i].y, init_yaw) + init_y;
		temp_point.z = pc.points[i].z + init_z;
		curvature_obs_221_.points.push_back(temp_point);
	}
	// cout<<"obs_220_.points.size() : "<<obs_220_.points.size()<<endl;
}

void curvature_obs_222_callback(const sensor_msgs::PointCloud2 msg){
	boost::mutex::scoped_lock(mutex_curvature_222);
	sensor_msgs::PointCloud2 pc2;
	sensor_msgs::PointCloud pc;
	pc2 = msg;
	sensor_msgs::convertPointCloud2ToPointCloud(pc2, pc);
	curvature_obs_222_.points.clear();
	// cout<<"pc.points.size() : "<<pc.points.size()<<endl;
	for(size_t i=0; i<pc.points.size();i++){
		// cout<<"i : "<<i<<endl;
		geometry_msgs::Point32 temp_point;
		temp_point.x = rotation_x(pc.points[i].x, pc.points[i].y, init_yaw) + init_x;
		temp_point.y = rotation_y(pc.points[i].x, pc.points[i].y, init_yaw) + init_y;
		temp_point.z = pc.points[i].z + init_z;
		curvature_obs_222_.points.push_back(temp_point);
	}
	// cout<<"obs_220_.points.size() : "<<obs_220_.points.size()<<endl;
}


void PC2GridMap(sensor_msgs::PointCloud& pc, nav_msgs::OccupancyGrid& map)
{
	int pt_size=pc.points.size();
	double x,y;
	int p,q,q_;

	for(size_t i=0;i<MAP_SIZE;i++){
		map.data[i] = _FREE;
	}

	for (int i=0; i<pt_size; ++i){
		x = pc.points[i].x;
		y = pc.points[i].y;

		p = (x+HALF_WIDTH)*INV_Resolution;
		q = (y+HALF_WIDTH)*INV_Resolution;
		q_ = q*MAP_WIDTH;

		if(fabs(x)<HALF_WIDTH && fabs(y)<HALF_WIDTH){
			map.data[ p + q_ ] = 100;
		}
	}
}

void map_integrator(nav_msgs::OccupancyGrid& output, 
		nav_msgs::OccupancyGrid& input1, 
		nav_msgs::OccupancyGrid& input2, 
		nav_msgs::OccupancyGrid& input3, 
		nav_msgs::OccupancyGrid& input4, 
		nav_msgs::OccupancyGrid& input5, 
		nav_msgs::OccupancyGrid& input6){
	for(size_t i=0;i<MAP_SIZE;i++){
		output.data[i] = _FREE;
	}

	for(size_t i=0; i<MAP_SIZE;i++){
		if(input1.data[i] != 0 || input2.data[i] != 0 || input3.data[i] != 0 || 
				input4.data[i] != 0 || input5.data[i] != 0 || input6.data[i] != 0){
			output.data[i] = 100;
		}
	}
}

// void map_integrator(nav_msgs::OccupancyGrid& output, 
		// nav_msgs::OccupancyGrid& input1, 
		// nav_msgs::OccupancyGrid& input2, 
		// nav_msgs::OccupancyGrid& input3){
	// for(size_t i=0;i<MAP_SIZE;i++){
		// output.data[i] = _FREE;
	// }

	// for(size_t i=0; i<MAP_SIZE;i++){
		// if(input1.data[i] != 0 || input2.data[i] != 0 || input3.data[i] != 0){
			// output.data[i] = 100;
		// }
	// }
// }

int main(int argc, char** argv){
	ros::init(argc, argv, "create_tom_laser_local_map");
	ros::NodeHandle n;
	
	ros::Subscriber sub_220 = n.subscribe("/point_cloud2/obstacle_220", 10, obs_220_callback);
	ros::Subscriber sub_221 = n.subscribe("/point_cloud2/obstacle_221", 10, obs_221_callback);
	ros::Subscriber sub_222 = n.subscribe("/point_cloud2/obstacle_222", 10, obs_222_callback);

	ros::Subscriber sub_curvature_220 = n.subscribe("/point_cloud2/curvature_obstacle_220", 10, curvature_obs_220_callback);
	ros::Subscriber sub_curvature_221 = n.subscribe("/point_cloud2/curvature_obstacle_221", 10, curvature_obs_221_callback);
	ros::Subscriber sub_curvature_222 = n.subscribe("/point_cloud2/curvature_obstacle_222", 10, curvature_obs_222_callback);

	ros::Publisher local_map_220_pub = n.advertise<nav_msgs::OccupancyGrid>("/local_map_real/tom_laser/220", 1);
	ros::Publisher local_map_221_pub = n.advertise<nav_msgs::OccupancyGrid>("/local_map_real/tom_laser/221", 1);
	ros::Publisher local_map_222_pub = n.advertise<nav_msgs::OccupancyGrid>("/local_map_real/tom_laser/222", 1);
	ros::Publisher local_map_curvature_220_pub = n.advertise<nav_msgs::OccupancyGrid>("/local_map_real/tom_laser/curvature_220", 1);
	ros::Publisher local_map_curvature_221_pub = n.advertise<nav_msgs::OccupancyGrid>("/local_map_real/tom_laser/curvature_221", 1);
	ros::Publisher local_map_curvature_222_pub = n.advertise<nav_msgs::OccupancyGrid>("/local_map_real/tom_laser/curvature_222", 1);
	
	ros::Publisher local_map_pub = n.advertise<nav_msgs::OccupancyGrid>("/local_map_real/tom_laser", 1);
	ros::Publisher expand_local_map_pub = n.advertise<nav_msgs::OccupancyGrid>("/local_map_real/tom_laser/expand", 1);
	
	obs_220_.header.frame_id = "/velodyne";
	obs_221_.header.frame_id = "/velodyne";
	obs_222_.header.frame_id = "/velodyne";
	curvature_obs_220_.header.frame_id = "/velodyne";
	curvature_obs_221_.header.frame_id = "/velodyne";
	curvature_obs_222_.header.frame_id = "/velodyne";

	nav_msgs::OccupancyGrid map;
	map.header.frame_id = "/velodyne";
	map.data.resize(MAP_SIZE);
	map.info.width = MAP_WIDTH;
	map.info.height = MAP_LENGTH;
	map.info.resolution = Resolution;
	map.info.origin.position.x = -Length/2.0;
	map.info.origin.position.y = -Width/2.0;

	nav_msgs::OccupancyGrid obs_220_map;
	obs_220_map.header.frame_id = "/velodyne";
	obs_220_map.data.resize(MAP_SIZE);
	obs_220_map.info.width = MAP_WIDTH;
	obs_220_map.info.height = MAP_LENGTH;
	obs_220_map.info.resolution = Resolution;
	obs_220_map.info.origin.position.x = -Length/2.0;
	obs_220_map.info.origin.position.y = -Width/2.0;

	nav_msgs::OccupancyGrid obs_221_map;
	obs_221_map.header.frame_id = "/velodyne";
	obs_221_map.data.resize(MAP_SIZE);
	obs_221_map.info.width = MAP_WIDTH;
	obs_221_map.info.height = MAP_LENGTH;
	obs_221_map.info.resolution = Resolution;
	obs_221_map.info.origin.position.x = -Length/2.0;
	obs_221_map.info.origin.position.y = -Width/2.0;

	nav_msgs::OccupancyGrid obs_222_map;
	obs_222_map.header.frame_id = "/velodyne";
	obs_222_map.data.resize(MAP_SIZE);
	obs_222_map.info.width = MAP_WIDTH;
	obs_222_map.info.height = MAP_LENGTH;
	obs_222_map.info.resolution = Resolution;
	obs_222_map.info.origin.position.x = -Length/2.0;
	obs_222_map.info.origin.position.y = -Width/2.0;

	nav_msgs::OccupancyGrid curvature_obs_220_map;
	curvature_obs_220_map.header.frame_id = "/velodyne";
	curvature_obs_220_map.data.resize(MAP_SIZE);
	curvature_obs_220_map.info.width = MAP_WIDTH;
	curvature_obs_220_map.info.height = MAP_LENGTH;
	curvature_obs_220_map.info.resolution = Resolution;
	curvature_obs_220_map.info.origin.position.x = -Length/2.0;
	curvature_obs_220_map.info.origin.position.y = -Width/2.0;

	nav_msgs::OccupancyGrid curvature_obs_221_map;
	curvature_obs_221_map.header.frame_id = "/velodyne";
	curvature_obs_221_map.data.resize(MAP_SIZE);
	curvature_obs_221_map.info.width = MAP_WIDTH;
	curvature_obs_221_map.info.height = MAP_LENGTH;
	curvature_obs_221_map.info.resolution = Resolution;
	curvature_obs_221_map.info.origin.position.x = -Length/2.0;
	curvature_obs_221_map.info.origin.position.y = -Width/2.0;

	nav_msgs::OccupancyGrid curvature_obs_222_map;
	curvature_obs_222_map.header.frame_id = "/velodyne";
	curvature_obs_222_map.data.resize(MAP_SIZE);
	curvature_obs_222_map.info.width = MAP_WIDTH;
	curvature_obs_222_map.info.height = MAP_LENGTH;
	curvature_obs_222_map.info.resolution = Resolution;
	curvature_obs_222_map.info.origin.position.x = -Length/2.0;
	curvature_obs_222_map.info.origin.position.y = -Width/2.0;
	
	ExpandMap ex_map(0.3, 0.1);

	ros::Rate loop_rate(20);

	while(ros::ok()){
		sensor_msgs::PointCloud obs_220;
		{
			boost::mutex::scoped_lock(mutex_220);
			obs_220 = obs_220_;
		}
		sensor_msgs::PointCloud obs_221;
		{
			boost::mutex::scoped_lock(mutex_221);
			obs_221 = obs_221_;
		}
		sensor_msgs::PointCloud obs_222;
		{
			boost::mutex::scoped_lock(mutex_222);
			obs_222 = obs_222_;
		}
		sensor_msgs::PointCloud curvature_obs_220;
		{
			boost::mutex::scoped_lock(curvature_mutex_220);
			curvature_obs_220 = curvature_obs_220_;
		}
		sensor_msgs::PointCloud curvature_obs_221;
		{
			boost::mutex::scoped_lock(curvature_mutex_221);
			curvature_obs_221 = curvature_obs_221_;
		}
		sensor_msgs::PointCloud curvature_obs_222;
		{
			boost::mutex::scoped_lock(curvature_mutex_222);
			curvature_obs_222 = curvature_obs_222_;
		}

		cout<<"obs_220.points.size() : "<<obs_220.points.size()<<endl;
		cout<<"obs_221.points.size() : "<<obs_221.points.size()<<endl;
		cout<<"obs_222.points.size() : "<<obs_222.points.size()<<endl;

		cout<<"curvature_obs_220.points.size() : "<<curvature_obs_220.points.size()<<endl;
		cout<<"curvature_obs_221.points.size() : "<<curvature_obs_221.points.size()<<endl;
		cout<<"curvature_obs_222.points.size() : "<<curvature_obs_222.points.size()<<endl;
		
		PC2GridMap(obs_220, obs_220_map);
		PC2GridMap(obs_221, obs_221_map);
		PC2GridMap(obs_222, obs_222_map);
		PC2GridMap(curvature_obs_220, curvature_obs_220_map);
		PC2GridMap(curvature_obs_221, curvature_obs_221_map);
		PC2GridMap(curvature_obs_222, curvature_obs_222_map);

		map_integrator(map, obs_220_map, obs_221_map, obs_222_map, curvature_obs_220_map, curvature_obs_221_map, curvature_obs_222_map);

		ex_map.expandObstacle(map);

		obs_220_map.header.stamp = ros::Time::now();
		local_map_220_pub.publish(obs_220_map);
		obs_221_map.header.stamp = ros::Time::now();
		local_map_221_pub.publish(obs_221_map);
		obs_222_map.header.stamp = ros::Time::now();
		local_map_222_pub.publish(obs_222_map);
		curvature_obs_220_map.header.stamp = ros::Time::now();
		local_map_curvature_220_pub.publish(curvature_obs_220_map);
		curvature_obs_221_map.header.stamp = ros::Time::now();
		local_map_curvature_221_pub.publish(curvature_obs_221_map);
		curvature_obs_222_map.header.stamp = ros::Time::now();
		local_map_curvature_222_pub.publish(curvature_obs_222_map);
		
		map.header.stamp = ros::Time::now();
		local_map_pub.publish(map);
		expand_local_map_pub.publish(ex_map.local_map);

		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}
