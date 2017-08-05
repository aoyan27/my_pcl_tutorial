#include<ros/ros.h>
#include<math.h>
#include<iostream>
#include<vector>
#include<boost/thread.hpp>
#include<sensor_msgs/PointCloud.h>
#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/point_cloud_conversion.h>
#include<nav_msgs/Odometry.h>
#include<pcl_ros/point_cloud.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#define MIN(x,y) ((x) < (y) ? (x) : (y))
#define MAX(x,y) ((x) > (y) ? (x) : (y))
#define cell_size 0.1
#define full_clouds true
#define grid_dimentions 100
#define height_threshold 0.20
#define max_height 1.5

#define LIDAR_POINTS 1081
#define SAVE_NUM 60

const float init_x = 0.0;
const float init_y = 0.28;
const float init_z = -0.4;
const float init_yaw = 90.0/180.0 * M_PI;

using namespace std;

////////////////////////////////////////////////
///////////// odometry callback/////////////////
////////////////////////////////////////////////
boost::mutex odom_mutex;
nav_msgs::Odometry ndt_odom;
void ndt_odom_callback(nav_msgs::Odometry msg){
	ndt_odom = msg;
	/*cout<<"ndt_odom : "<<ndt_odom.pose.pose.orientation.x<<endl;
	cout<<"ndt_odom : "<<ndt_odom.pose.pose.orientation.y<<endl;
	cout<<"ndt_odom : "<<ndt_odom.pose.pose.orientation.z<<endl;
	cout<<"ndt_odom : "<<ndt_odom.pose.pose.orientation.w<<endl;*/
}

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> CloudT;
pcl::PointCloud<pcl::PointXYZ> pcl_in;
pcl::PointCloud<pcl::PointXYZ> pcl_velodyne;
//pcl::PointCloud<pcl::PointXYZ> pcl_odom;
pcl::PointCloud<pcl::PointXYZ> pcl_out;
/////////////////////////////////////////////////
///////////// pc callback ///////////////////////
/////////////////////////////////////////////////
boost::mutex pc_mutex;
ros::Publisher pub_obstacle;
ros::Publisher pub_empty;
int t;
int pc_count = 0;
double theta;

////////////////////////////////////////////////
//////////////// min_max ///////////////////////
///////////////////////////////////////////////
//int obstacle_count=0;
//int empty_count=0;
bool save = false;

//////////////////////////////////////////////////
///////////// constract full clouds //////////////
//////////////////////////////////////////////////
void constructFullClouds(const CloudT& pcl_in, CloudT&pcl_obstacle, CloudT& pcl_empty, const CloudT& pcl_velodyne)
{
        float min[grid_dimentions][grid_dimentions];
        float max[grid_dimentions][grid_dimentions];
        bool init[grid_dimentions][grid_dimentions];
        memset(&init,0,grid_dimentions*grid_dimentions);
        size_t npoints = pcl_in.points.size();

        for(unsigned i=0;i<npoints;i++)
        {
                int x = ((grid_dimentions/2)+pcl_in.points[i].x/cell_size);
                int y = ((grid_dimentions/2)+pcl_in.points[i].y/cell_size);

                if( x >= 0 && x < grid_dimentions && y >= 0 && y < grid_dimentions){
                        if(!init[x][y]){
                                min[x][y]  = pcl_in.points[i].z;
                                max[x][y]  = pcl_in.points[i].z;
                                init[x][y] = true;
                        }
                        else{
                                min[x][y] = MIN(min[x][y],pcl_in.points[i].z);
                                max[x][y] = MAX(max[x][y],pcl_in.points[i].z);
                        }
                }
        }

        for(unsigned i=0;i<npoints;i++)
        {
                int x = ((grid_dimentions/2)+pcl_in.points[i].x/cell_size);
                int y = ((grid_dimentions/2)+pcl_in.points[i].y/cell_size);
                double range = sqrt(pcl_in.points[i].x*pcl_in.points[i].x + pcl_in.points[i].y*pcl_in.points[i].y);
                double oval = (pcl_in.points[i].x * pcl_in.points[i].x)/(0.35 * 0.35) + (pcl_in.points[i].y * pcl_in.points[i].y)/(0.40 * 0.40);

                if( x >= 0 && x < grid_dimentions && y >= 0 && y < grid_dimentions && init[x][y])
                {
                        if(( max[x][y]-min[x][y] > height_threshold) && ( max[x][y]-min[x][y] < max_height) && (pcl_in.points[i].z < 0.4)  && (pcl_in.points[i].x >= 0.1) && (pcl_in.points[i].z >= -0.85) && (oval>1))
                        {
                                pcl::PointXYZ p;
                                p.x=pcl_velodyne.points[i].x;
                                p.y=pcl_velodyne.points[i].y;
                                p.z=pcl_velodyne.points[i].z;
                                //p.x=pcl_odom.points[i].x;
                                //p.y=pcl_odom.points[i].y;
                                //p.z=pcl_odom.points[i].z;
                                pcl_obstacle.push_back(p);
                        }
                        else{ 
                                pcl::PointXYZ q;
                                q.x=pcl_velodyne.points[i].x;
                                q.y=pcl_velodyne.points[i].y;
                                q.z=pcl_velodyne.points[i].z;
                                pcl_empty.push_back(q);
                        }
                }
        }
}
void pcCallback(const sensor_msgs::PointCloudConstPtr& msg)
{
        //////////////////////////////////////
        //////////// transform ///////////////
        //////////////////////////////////////
        boost::mutex::scoped_lock(pc_mutex);
        sensor_msgs::PointCloud pc_callback = *msg;
        t = pc_count*LIDAR_POINTS;
        theta = ndt_odom.pose.pose.orientation.z;

        for(int i=0;i<pc_callback.points.size();i++){
                if(pc_callback.points[i].x == 0 && pc_callback.points[i].y==0 && pc_callback.points[i].z==0){
                        pcl_in.points[i+t].x = 0.0;
                        pcl_in.points[i+t].y = 0.0;
                        pcl_in.points[i+t].z = 0.0;

                        pcl_velodyne.points[i+t].x = 0.0;
                        pcl_velodyne.points[i+t].y = 0.0;
                        pcl_velodyne.points[i+t].z = 0.0;

                        //pcl_odom.points[i+t].x = 0.0;
                        //pcl_odom.points[i+t].y = 0.0;
                        //pcl_odom.points[i+t].z = 0.0;
                }
                else{
                        pcl_in.points[i+t].x = pc_callback.points[i].x;
                        pcl_in.points[i+t].y = pc_callback.points[i].y;
                        pcl_in.points[i+t].z = pc_callback.points[i].z;

                        pcl_velodyne.points[i+t].x = pc_callback.points[i].x*cos(init_yaw) - pc_callback.points[i].y*sin(init_yaw) + init_x;
                        pcl_velodyne.points[i+t].y = pc_callback.points[i].x*sin(init_yaw) + pc_callback.points[i].y*cos(init_yaw) + init_y;
                        pcl_velodyne.points[i+t].z = pc_callback.points[i].z + init_z;

                        //pcl_odom.points[i+t].x = pcl_velodyne.points[i+t].x * cos(theta) - pcl_velodyne.points[i+t].y * sin(theta) + ndt_odom.pose.pose.position.x;
                        //pcl_odom.points[i+t].y = pcl_velodyne.points[i+t].x * sin(theta) - pcl_velodyne.points[i+t].y * cos(theta) + ndt_odom.pose.pose.position.y;
                        //pcl_odom.points[i+t].z = pcl_velodyne.points[i+t].z;

                }
        }

        if(save){
                pcl::PointCloud<pcl::PointXYZ> pcl_obstacle;
                pcl::PointCloud<pcl::PointXYZ> pcl_empty;
                constructFullClouds(pcl_in, pcl_obstacle, pcl_empty,pcl_velodyne);
                
                sensor_msgs::PointCloud2 ros_out_obstacle;
                pcl::toROSMsg(pcl_obstacle,ros_out_obstacle);
                ros_out_obstacle.header.frame_id="velodyne";
                ros_out_obstacle.header.stamp= ros::Time::now();
                
                sensor_msgs::PointCloud2 ros_out_empty;
                pcl::toROSMsg(pcl_empty,ros_out_empty);
                ros_out_empty.header.frame_id="velodyne";
                ros_out_empty.header.stamp=ros::Time::now();

                pub_obstacle.publish(ros_out_obstacle);
                pub_empty.publish(ros_out_empty);

                //cout<<"pcl_obstacle.points:"<<pcl_obstacle.points.size()<<endl;
                //cout<<"pcl_empty.points:"<<pcl_empty.points.size()<<endl;

                //obstacle_count = 0;
                //empty_count = 0;

        }


        ++pc_count;
        if(pc_count == SAVE_NUM){
                save=true;
                pc_count=0;
                cout<<"save"<<endl;
        }
}

int main(int argc,char** argv)
{
        ros::init(argc,argv,"infant_height_map_odom_221");
        ros::NodeHandle n;

        ros::Subscriber sub = n.subscribe("eth_221/point_cloud_3",10,pcCallback);
        ros::Subscriber sub_odo = n.subscribe("/ekf_DgaussAndNDT", 1 , ndt_odom_callback);

        pub_obstacle   = n.advertise<sensor_msgs::PointCloud2>("point_cloud2/obstacle_221",10);
        pub_empty      = n.advertise<sensor_msgs::PointCloud2>("point_cloud2/empty_221",10);

        pcl_in.header.frame_id="3dlaser";
        pcl_in.points.resize(SAVE_NUM*LIDAR_POINTS);

        pcl_velodyne.points.resize(SAVE_NUM*LIDAR_POINTS);

        //pcl_odom.header.frame_id = "map";
        //pcl_odom.points.resize(SAVE_NUM*LIDAR_POINTS);

        ros::Rate rate(40);

        while(ros::ok())
        {
                ros::spinOnce();
                rate.sleep();
        }
        return 0;
}
