#include<ros/ros.h>
#include<math.h>
#include<iostream>
#include<vector>
#include<boost/thread.hpp>
#include<sensor_msgs/PointCloud.h>
#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/point_cloud_conversion.h>
#include<nav_msgs/Odometry.h>
#include<knm_tiny_msgs/Velocity.h>
#include<pcl_ros/point_cloud.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#define MIN(x,y) ((x) < (y) ? (x) : (y))
#define MAX(x,y) ((x) > (y) ? (x) : (y))
#define cell_size 0.1
#define full_clouds true
#define grid_dimentions 100
#define height_threshold 0.30
#define max_height 1.5


#define LIDAR_POINTS 1081
#define SAVE_NUM 60

using namespace std;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> CloudT;
pcl::PointCloud<pcl::PointXYZ> pcl_in;
pcl::PointCloud<pcl::PointXYZ> pcl_out;
/////////////////////////////////////////////////
///////////// pc callback ///////////////////////
/////////////////////////////////////////////////
boost::mutex pc_mutex;
ros::Publisher pub_obstacle;
ros::Publisher pub_empty;
int t;
int pc_count = 0;

////////////////////////////////////////////////
//////////////// min_max ///////////////////////
///////////////////////////////////////////////
//int obstacle_count=0;
//int empty_count=0;
bool save = false;

//////////////////////////////////////////////////
///////////// constract full clouds //////////////
//////////////////////////////////////////////////
void constructFullClouds(const CloudT& pcl_in, CloudT&pcl_obstacle, CloudT& pcl_empty)
{
        float min[grid_dimentions][grid_dimentions];
        float max[grid_dimentions][grid_dimentions];
        bool init[grid_dimentions][grid_dimentions];
        memset(&init,0,grid_dimentions*grid_dimentions);
        size_t npoints = pcl_in.points.size();

        //cout<<"build height map"<<endl;
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

        //cout<<"display poitns where map has height-difference > threshold"<<endl;
        for(unsigned i=0;i<npoints;i++)
        {
                //cout<<"i:"<<i<<endl;
                int x = ((grid_dimentions/2)+pcl_in.points[i].x/cell_size);
                int y = ((grid_dimentions/2)+pcl_in.points[i].y/cell_size);
                //int r = sqrt((pcl_in.points[i].x+0.3)*(pcl_in.points[i].x+0.3) + pcl_in.points[i].y*pcl_in.points[i].y);
                int r = sqrt((pcl_in.points[i].x+0.0)*(pcl_in.points[i].x+0.0) + pcl_in.points[i].y*pcl_in.points[i].y);
                //double oval = ((pcl_in.points[i].x+0.30) * (pcl_in.points[i].x+0.30))/(0.50 * 0.50) + (pcl_in.points[i].y * pcl_in.points[i].y)/(0.50 * 0.50);
                double oval = ((pcl_in.points[i].x+0.2) * (pcl_in.points[i].x+0.20))/(0.40 * 0.40) + (pcl_in.points[i].y * pcl_in.points[i].y)/(0.50 * 0.50);
                if( x >= 0 && x < grid_dimentions && y >= 0 && y < grid_dimentions && init[x][y])
                {
                         if( ( max[x][y]-min[x][y] > height_threshold) && ( max[x][y]-min[x][y] < max_height) && (-0.85 < pcl_in.points[i].z) && (pcl_in.points[i].z < 0.4) && (r > 0.1))
                        {
                                pcl::PointXYZ p;
                                p.x=pcl_in.points[i].x;
                                p.y=pcl_in.points[i].y;
                                p.z=pcl_in.points[i].z;
                                pcl_obstacle.push_back(p);
                                //obstacle_count++;
                        }
                        else{ 
                                pcl::PointXYZ q;
                                q.x=pcl_in.points[i].x;
                                q.y=pcl_in.points[i].y;
                                q.z=pcl_in.points[i].z;
                                pcl_empty.push_back(q);
                                //empty_count++;
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

        for(int i=0;i<pc_callback.points.size();i++){
                if(pc_callback.points[i].x == 0 && pc_callback.points[i].y==0 && pc_callback.points[i].z==0){
                        pcl_in.points[i+t].x = 0.0;
                        pcl_in.points[i+t].y = 0.0;
                        pcl_in.points[i+t].z = 0.0;
                }
                else{
                        pcl_in.points[i+t].x = pc_callback.points[i].x;
                        pcl_in.points[i+t].y = pc_callback.points[i].y;
                        pcl_in.points[i+t].z = pc_callback.points[i].z;
                }
        }

        if(save){
                pcl::PointCloud<pcl::PointXYZ> pcl_obstacle;
                pcl::PointCloud<pcl::PointXYZ> pcl_empty;
                constructFullClouds(pcl_in, pcl_obstacle, pcl_empty);
                
                sensor_msgs::PointCloud2 ros_out_obstacle;
                pcl::toROSMsg(pcl_obstacle,ros_out_obstacle);
                ros_out_obstacle.header.frame_id="3dlaser";
                // ros_out_obstacle.header.stamp= msg->header.stamp;
                ros_out_obstacle.header.stamp = ros::Time::now();
                
                sensor_msgs::PointCloud2 ros_out_empty;
                pcl::toROSMsg(pcl_empty,ros_out_empty);
                ros_out_empty.header.frame_id="3dlaser";
                // ros_out_empty.header.stamp=msg->header.stamp;
                ros_out_empty.header.stamp=ros::Time::now();
                
                //pcl_obstacle.header.frame_id="3dlaser";
                //pcl_obstacle.points.resize(obstacle_count);
                //pcl_obstacle.header.frame_id="3dlaser";
                //pcl_empty.points.resize(empty_count);
                pub_obstacle.publish(ros_out_obstacle);
                pub_empty.publish(ros_out_empty);
                //obstacle_count = 0;
                //empty_count = 0;

                /*if(pc_count==30){
                  cout<<"obstacle:"<<pcl_obstacle.points.size()<<endl;
                  cout<<"empty   :"<<pcl_empty.points.size()<<endl;
                  }*/
                //pcl_obstacle.points.clear();
                //pcl_empty.points.clear();
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
        ros::init(argc,argv,"infant_height_map_222");
        ros::NodeHandle n;

        ros::Subscriber sub = n.subscribe("eth_222/point_cloud_3",10,pcCallback);

        pub_obstacle   = n.advertise<sensor_msgs::PointCloud2>("point_cloud2/obstacle_222",10);
        pub_empty      = n.advertise<sensor_msgs::PointCloud2>("point_cloud2/empty_222",10);

        pcl_in.header.frame_id="3dlaser";
        pcl_in.points.resize(SAVE_NUM*LIDAR_POINTS);

        ros::Rate rate(40);

        while(ros::ok())
        {
                ros::spinOnce();
                rate.sleep();
        }
        return 0;
}

