#include<ros/ros.h>
#include<iostream>
#include<math.h>
#include<boost/thread.hpp>
#include<geometry_msgs/Point32.h>
#include<sensor_msgs/PointCloud.h>
#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/point_cloud_conversion.h>
#include<pcl/point_types.h>
#include<pcl/features/normal_3d.h>
#include<pcl_ros/point_cloud.h>

#define SAVE_NUM 40
#define LIDAR_POINTS 1081
#define SAVE_CURVATURE 4

const float init_x = 0.0;
const float init_y = 0.28;
const float init_z = -0.4;
const float init_yaw = 90.0/180.0 * M_PI;

using namespace std;
ros::Publisher pub_visualize;
ros::Publisher pub_obstacle;
ros::Publisher pub_empty;

/////////////////////////////////////////////////
/////////// normal estimation ///////////////////
/////////////////////////////////////////////////
pcl::PointCloud<pcl::PointNormal> curvature_estimation(pcl::PointCloud<pcl::PointXYZ> cloud)
{
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>);
        *cloud_input = cloud;

        pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;
        ne.setInputCloud(cloud_input);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        ne.setRadiusSearch(0.1);
        //ne.setRadiusSearch(0.3);
        ne.compute(*cloud_normals);

        int num_pt=cloud_input->points.size();
        pcl::PointCloud<pcl::PointNormal> pcl_normal;
        pcl_normal.points.resize(num_pt);
        for(int i=0;i<num_pt;i++){
                pcl_normal.points[i].x         = cloud_input->points[i].x;
                pcl_normal.points[i].y         = cloud_input->points[i].y;
                pcl_normal.points[i].z         = cloud_input->points[i].z;
                pcl_normal.points[i].normal_x  = cloud_normals->points[i].normal_x;
                pcl_normal.points[i].normal_y  = cloud_normals->points[i].normal_y;
                pcl_normal.points[i].normal_z  = cloud_normals->points[i].normal_z;
                pcl_normal.points[i].curvature = cloud_normals->points[i].curvature;
        }

        return pcl_normal;
}

/////////////////////////////////////////////////
//////////// sensor_callback ////////////////////
/////////////////////////////////////////////////
boost::mutex pc_mutex;
sensor_msgs::PointCloud pc_in;
sensor_msgs::PointCloud2 ros_out_obstacle;
sensor_msgs::PointCloud2 ros_out_empty;
pcl::PointCloud<pcl::PointXYZ> pcl_in;
pcl::PointCloud<pcl::PointXYZ> pcl_velodyne;
pcl::PointCloud<pcl::PointXYZRGBA> pcl_visualize;

int t;
int h;
int pc_count = 0;
bool save_flag = false;

void pcCallback(const sensor_msgs::PointCloudConstPtr& msg)
{
        pc_in = *msg;
        boost::mutex::scoped_lock(pc_mutex);
        t = pc_count*LIDAR_POINTS;

        for(size_t i=0;i<LIDAR_POINTS;i++){
                if(pc_in.points[i].x==0 && pc_in.points[i].y==0 && pc_in.points[i].z == 0){
                        pcl_in.points[i+t].x = 0.0;
                        pcl_in.points[i+t].y = 0.0;
                        pcl_in.points[i+t].z = 0.0;
                        pcl_velodyne.points[i+t].x = 0.0;
                        pcl_velodyne.points[i+t].y = 0.0;
                        pcl_velodyne.points[i+t].z = 0.0;

                }else{
                        pcl_in.points[i+t].x = pc_in.points[i].x;
                        pcl_in.points[i+t].y = pc_in.points[i].y;
                        pcl_in.points[i+t].z = pc_in.points[i].z;

                        pcl_velodyne.points[i+t].x = pc_in.points[i].x*cos(init_yaw) - pc_in.points[i].y*sin(init_yaw) + init_x;
                        pcl_velodyne.points[i+t].y = pc_in.points[i].x*sin(init_yaw) + pc_in.points[i].y*cos(init_yaw) + init_y;
                        pcl_velodyne.points[i+t].z = pc_in.points[i].z + init_z;

                }
        }

        if(save_flag && pc_count % SAVE_CURVATURE == 0){

                //cout<<"pc_count:"<<pc_count<<endl;

                size_t save_hz = SAVE_CURVATURE*LIDAR_POINTS;

                pcl::PointCloud<pcl::PointXYZRGBA> pcl_obstacle;
                pcl::PointCloud<pcl::PointXYZRGBA> pcl_empty;

                //cout<<"pcl save"<<endl;
                pcl::PointCloud<pcl::PointXYZ> pcl_save;
                pcl_save.points.resize(SAVE_CURVATURE*LIDAR_POINTS);
                for(size_t i=0;i<save_hz;i++){
                        //pcl_save.points[i] = pcl_in.points[i+t];
                        pcl_save.points[i] = pcl_velodyne.points[i+t];
                }

                //cout<<"curvature estimation"<<endl;
                pcl::PointCloud<pcl::PointNormal> pcl_curvature;
                pcl_curvature.points.resize(save_hz);
                pcl_curvature = curvature_estimation(pcl_save);

                //cout<<"segmentation"<<endl;
                for(size_t i=0;i<pcl_curvature.points.size();i++){

                        double oval = ( (pcl_curvature.points[i].x) * (pcl_curvature.points[i].x))/(0.5*0.5) + ((pcl_curvature.points[i].y+0.2) * (pcl_curvature.points[i].y+0.2))/(0.80 * 0.80);

                        pcl_visualize.points[i+t].x = pcl_curvature.points[i].x;
                        pcl_visualize.points[i+t].y = pcl_curvature.points[i].y;
                        pcl_visualize.points[i+t].z = pcl_curvature.points[i].z;
                        
                        if( ( -1.15 < pcl_curvature.points[i].z) && (pcl_curvature.points[i].z <0.20) && (oval > 1) && (pcl_curvature.points[i].curvature > 0.10))
                        {
                                pcl_visualize.points[i+t].r = 0;
                                pcl_visualize.points[i+t].g = 0;
                                pcl_visualize.points[i+t].b = 255;
                                pcl_visualize.points[i+t].a = 255;
                        }else{
                                pcl_visualize.points[i+t].r = 255;
                                pcl_visualize.points[i+t].g = 255;
                                pcl_visualize.points[i+t].b = 255;
                                pcl_visualize.points[i+t].a = 50;
                        }
                }
                //cout<<"obstacle or empty"<<endl;
                for(size_t i=0;i<SAVE_NUM*LIDAR_POINTS;i++)
                {
                        if(pcl_visualize.points[i].a == 255)
                        {
                                pcl::PointXYZRGBA p;
                                p=pcl_visualize.points[i];
                                pcl_obstacle.push_back(p);
                        }else{
                                pcl::PointXYZRGBA q;
                                q=pcl_visualize.points[i];
                                pcl_empty.push_back(q);
                        }
                }
                //cout<<"publish"<<endl;
                pcl::toROSMsg(pcl_obstacle,ros_out_obstacle);
                pcl::toROSMsg(pcl_empty,ros_out_empty);

                ros_out_obstacle.header.frame_id="velodyne";
                ros_out_empty.header.frame_id="velodyne";

                pub_obstacle.publish(ros_out_obstacle);
                pub_empty.publish(ros_out_empty);
        }

        ++pc_count;

        if(pc_count == SAVE_NUM)
        {
                cout<<"save"<<endl;
                save_flag = true;
                pc_count = 0;
        }
}

int main(int argc,char** argv)
{
        ros::init(argc,argv,"infant_curvature_odom_221");
        ros::NodeHandle n;

        pcl_in.points.resize(SAVE_NUM*LIDAR_POINTS);
        pcl_visualize.points.resize(SAVE_NUM*LIDAR_POINTS);
        pcl_velodyne.points.resize(SAVE_NUM*LIDAR_POINTS);

        ros::Subscriber sub = n.subscribe("eth_221/point_cloud_3",10,pcCallback);

        pub_obstacle = n.advertise<sensor_msgs::PointCloud2>("point_cloud2/curvature_obstacle_221",10);
        pub_empty = n.advertise<sensor_msgs::PointCloud2>("point_cloud2/curvature_empty_221",10);

        ros::Rate rate(40);
        while(ros::ok())
        {
                ros::spinOnce();
                rate.sleep();
        }

        return 0;
}

