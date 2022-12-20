#include <iostream>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/message_instance.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "save_pcd");
    ros::NodeHandle nh("~");

    string bag_path;
    string pcd_path;
    string lidar_topic;
    int frame = 20;
    int x_maxRange = 100;

    nh.getParam("bag_path", bag_path);
    nh.getParam("pcd_path", pcd_path);
    nh.getParam("lidar_topic", lidar_topic);
    nh.getParam("frame", frame);
    nh.getParam("x_maxRange", x_maxRange);

    cout << "bag_path:" << bag_path << endl;
    cout << "pcd_path:" << pcd_path << endl;
    cout << "lidar_topic:" << lidar_topic << endl;
    cout << "frame:" << frame << endl;
    cout << "x_maxRange:" << x_maxRange << endl;

    rosbag::Bag bag;
    bag.open(bag_path);
    vector<string> topics;
    topics.emplace_back(lidar_topic);
    rosbag::View view_pcd(bag, rosbag::TopicQuery(topics));
    int index = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const rosbag::MessageInstance &m : view_pcd) {
        if (lidar_topic == m.getTopic()) {
            ++index;
            if (index < frame)
                continue;
            sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
            pcl::fromROSMsg(*msg, *cloud);
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < cloud->size(); ++i) {
        if (cloud->at(i).x < 0 || cloud->at(i).x > x_maxRange)
            continue;
        cloud_filter->push_back(cloud->at(i));
    }

    pcl::io::savePCDFileBinary(pcd_path, *cloud_filter);
    bag.close();

    return 0;
}