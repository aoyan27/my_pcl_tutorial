#include<ros/ros.h>
#include<iostream>
#include<math.h>
#include<boost/thread.hpp>
#include<geometry_msgs/Point32.h>
#include<sensor_msgs/PointCloud.h>
#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/point_cloud_conversion.h>
#include<nav_msgs/Odometry.h>
#include<pcl/point_types.h>
#include<pcl/features/normal_3d.h>
#include<pcl_ros/point_cloud.h>

#define SAVE_NUM 80
#define LIDAR_POINTS 1081
#define SAVE_CURVATURE 4

using namespace std;
ros::Publisher pub_cur;
ros::Publisher pub_real;

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
sensor_msgs::PointCloud pc_save;
sensor_msgs::PointCloud2 pc2_in;
sensor_msgs::PointCloud pc_tf;
sensor_msgs::PointCloud2 ros_out;
sensor_msgs::PointCloud2 ros_curvature;
pcl::PointCloud<pcl::PointXYZ> pcl_from;
pcl::PointCloud<pcl::PointNormal> curvature;
pcl::PointCloud<pcl::PointNormal> curvature_save;
//pcl::PointCloud<pcl::PointNormal> curvature_visualize;
//pcl::PointCloud<pcl::PointXYZRGBNormal> curvature_visualize;
pcl::PointCloud<pcl::PointXYZRGBA> curvature_visualize;

int t;
int h;
int pc_count=0;
int save_hz=0;
bool save_flag = false;

void callback(const sensor_msgs::PointCloudConstPtr& msg)
{
        pc_in = *msg;

        //座標変換
        boost::mutex::scoped_lock(pc_mutex);
        t = pc_count*1081;

        for(size_t i=0; i<LIDAR_POINTS; ++i){
                if(pc_in.points[i].x == 0 && pc_in.points[i].y==0 && pc_in.points[i].z==0){
                        pc_tf.points[i+t].x = 0.0;
                        pc_tf.points[i+t].y = 0.0;
                        pc_tf.points[i+t].z = 0.0;
                }else{
                        pc_tf.points[i+t].x = pc_in.points[i].x;
                        pc_tf.points[i+t].y = pc_in.points[i].y;
                        pc_tf.points[i+t].z = pc_in.points[i].z;
                        /*pc_tf.points[i+t].x = pc_in.points[i].x*cos_theta - pc_in.points[i].y*sin_theta;
                        pc_tf.points[i+t].y = pc_in.points[i].x*sin_theta + pc_in.points[i].y*cos_theta;
                        pc_tf.points[i+t].z = pc_in.points[i].z;

                        pc_tf.points[i+t].x += odom_in.pose.pose.position.x + CONICAL_X*cos_theta;
                        pc_tf.points[i+t].y += odom_in.pose.pose.position.y + CONICAL_X*sin_theta;
                        pc_tf.points[i+t].z += CONICAL_Z;
                        pc_tf.channels[0].values[i+t] = pc_in.channels[0].values[i];*/
                }
        }

        //SAVE_CURVATURE ごとにcurvature_estimation
        if(save_flag && pc_count % SAVE_CURVATURE ==0){
                //cout<<"pc_count:"<<pc_count<<endl;
                int save_hz = SAVE_CURVATURE*LIDAR_POINTS;
                for(int i=0;i<save_hz;i++){
                        pc_save.points[i]=pc_tf.points[i+t];
                }
                //from pointcloud to pointcloud2
                sensor_msgs::convertPointCloudToPointCloud2(pc_save,pc2_in);
                //pointcloud2 to pcl
                pcl::fromROSMsg(pc2_in,pcl_from);
                //curvature_estimation
                curvature = curvature_estimation(pcl_from);

                int size=curvature.points.size();
                //cout<<"curvature point size"<<size<<endl;
                for(int i=0;i<curvature.points.size();i++)
                {
                        curvature_save.points[i+t]=curvature.points[i];
                }

                //曲率が大きな所のみを表示
                for(int i=0;i<curvature.points.size();i++)
                {
                        double range = sqrt((curvature_save.points[i+t].x+0.2) * (curvature_save.points[i+t].x+0.2) + curvature_save.points[i+t].y * curvature_save.points[i+t].y);
                        double oval =( (curvature_save.points[i+t].x+0.4) * (curvature_save.points[i+t].x+0.4))/(0.6*0.6) + (curvature_save.points[i+t].y * curvature_save.points[i+t].y)/(0.50 * 0.50);
                        curvature_visualize.points[i+t].x = curvature_save.points[i+t].x;
                        curvature_visualize.points[i+t].y = curvature_save.points[i+t].y;
                        curvature_visualize.points[i+t].z = curvature_save.points[i+t].z;
                        //curvature_visualize.points[i+t].normal_x = curvature_save.points[i+t].normal_x;
                        //curvature_visualize.points[i+t].normal_y = curvature_save.points[i+t].normal_y;
                        //curvature_visualize.points[i+t].normal_z = curvature_save.points[i+t].normal_z;
                        //if(curvature_save.points[i+t].curvature > 0.01 )//&& hight >= 0.05 && 0.80 >= hight)
                        if( ( -0.75 < curvature_save.points[i+t].z) && (curvature_save.points[i+t].z <0.40) && (oval > 1) && (curvature_save.points[i+t].curvature > 0.06))//&& hight >= 0.05 && 0.80 >= hight)
                        {
                                //curvature_visualize.points[i+t].curvature = curvature_save.points[i+t].curvature;
                                curvature_visualize.points[i+t].r=0;
                                curvature_visualize.points[i+t].g=0;
                                curvature_visualize.points[i+t].b=255;
                                curvature_visualize.points[i+t].a=255;
                        }
                        else{
                                //curvature_visualize.points[i+t].curvature = 0.0;
                                curvature_visualize.points[i+t].r=255;
                                curvature_visualize.points[i+t].g=255;
                                curvature_visualize.points[i+t].b=255;
                                curvature_visualize.points[i+t].a=50;
                        }
                }

                //pcl to pointcloud2
                pcl::toROSMsg(curvature_visualize,ros_out);
                pcl::toROSMsg(curvature_save,ros_curvature);
                //frame_id
                ros_out.header.frame_id="3dlaser";
                ros_curvature.header.frame_id="3dlaser";

                //publish
				// ros_out.header.stamp = ros::Time::now();
                pub_cur.publish(ros_out);
				// ros_curvature.header.stamp = ros::Time::now();
                pub_real.publish(ros_curvature);
        }

        ++pc_count;

        if(pc_count == SAVE_NUM)
        {
                cout<<"save"<<endl;
                save_flag = true;
                pc_count = 0;
        } 
}


int main(int argc,char **argv)
{
        ros::init(argc,argv,"infant_curvature_222");
        ros::NodeHandle n;
        pc_tf.header.frame_id ="3dlasesr";
        pc_tf.channels.resize(1);
        pc_tf.channels[0].name = "intensity";
        pc_tf.points.resize(SAVE_NUM*LIDAR_POINTS);
        pc_tf.channels[0].values.resize(SAVE_NUM*LIDAR_POINTS);

        pc_save.header.frame_id="3dlaser";
        pc_save.points.resize(SAVE_CURVATURE*LIDAR_POINTS);

        pc2_in.header.frame_id="3dlasesr";

        pcl_from.header.frame_id="3dlaser";
        pcl_from.points.resize(SAVE_CURVATURE*LIDAR_POINTS);

        curvature.header.frame_id="3dlaser";
        curvature.points.resize(SAVE_CURVATURE*LIDAR_POINTS);

        curvature_save.header.frame_id="3dlasesr";
        curvature_save.points.resize(SAVE_NUM*LIDAR_POINTS);

        curvature_visualize.header.frame_id="3dlaser";
        curvature_visualize.resize(SAVE_NUM*LIDAR_POINTS);

        ros::Subscriber sub = n.subscribe("eth_222/point_cloud_3",10,callback);
        pub_cur = n.advertise<sensor_msgs::PointCloud2>("point_cloud2/curvature_obstacle_222",10);
        pub_real = n.advertise<sensor_msgs::PointCloud2>("point_cloud2/curvature_real_222",10);
        ros::Rate rate(40);
        while(ros::ok())
        {
                ros::spinOnce();
                rate.sleep();
        }
        return 0;
}
