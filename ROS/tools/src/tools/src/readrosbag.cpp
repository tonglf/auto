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

void handleLidarPcd(sensor_msgs::PointCloud2::ConstPtr msg, ros::Publisher &pub);

int main(int argc, char** argv) {
    ros::init(argc, argv, "readrosbag");
    ros::NodeHandle nh("~");

    string bag_path;
    string lidar_topic;
    nh.getParam("bag_path", bag_path);
    nh.getParam("lidar_topic", lidar_topic);

    cout << "bag_path: " << bag_path << endl;
    cout << "lidar_topic: " << lidar_topic << endl;

    ros::Publisher pcd_pub = nh.advertise<sensor_msgs::PointCloud2>("pcd", 1, false);

    rosbag::Bag bag;
    bag.open(bag_path);

    ros::Rate loop_rate(1);

    vector<string> topics;
    topics.emplace_back(lidar_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    for (const rosbag::MessageInstance &m : view) {
        if (lidar_topic == m.getTopic()) {
            sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
            handleLidarPcd(msg, pcd_pub);
        }

        loop_rate.sleep();
    }
    bag.close();

    return 0;
}

void handleLidarPcd(sensor_msgs::PointCloud2::ConstPtr msg, ros::Publisher &pub) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcd(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pcd);

    // ...
    cout << "dael with pointcloud!" <<endl;

    sensor_msgs::PointCloud2 ros_pcd;
    pcl::toROSMsg(*pcd, ros_pcd);
    
    ros_pcd.header.frame_id = "base_link";
    ros_pcd.header.stamp = ros::Time::now();


    pub.publish(ros_pcd);
}