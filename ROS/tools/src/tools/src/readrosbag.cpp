#include <iostream>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/message_instance.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

void handleLidarPcd(sensor_msgs::PointCloud2::ConstPtr msg);

int main(int argc, char** argv) {
    ros::init(argc, argv, "readrosbag");
    ros::NodeHandle nh("readrosbag");

    string bag_path;
    string pointcloud_topic;
    nh.getParam("bag_path", bag_path);
    nh.getParam("pointcloud_topic", pointcloud_topic);

    rosbag::Bag bag;
    bag.open(bag_path);

    vector<string> topics;
    topics.emplace_back(pointcloud_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    for (const rosbag::MessageInstance &m : view) {
        if (pointcloud_topic == m.getTopic()) {
            sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
            handleLidarPcd(msg);
        }
    }
    bag.close();
    
    return 0;
}

void handleLidarPcd(sensor_msgs::PointCloud2::ConstPtr msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcd(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pcd);


}