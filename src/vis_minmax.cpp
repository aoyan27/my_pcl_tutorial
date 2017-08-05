#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

sensor_msgs::PointCloud2 pc;

ros::Publisher vis_pub;


void rm_ground_callback(const sensor_msgs::PointCloud2 msg){
	pc = msg;
	pc.header.frame_id = "/velodyne";
	pc.header.stamp = ros::Time::now();
	vis_pub.publish(pc);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "vis_minmax");
	ros::NodeHandle n;

	ros::Subscriber sub_220 = n.subscribe("/rm_ground2", 10, rm_ground_callback);

	vis_pub = n.advertise<sensor_msgs::PointCloud2>("/rm_ground2/vis", 1);
	
	ros::spin();
	return 0;
}
