#include <iostream>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/message_instance.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "merge_rosbag");
    ros::NodeHandle nh("~");

    rosbag::Bag bag_read_pcd;
    rosbag::Bag bag_read_img;
    rosbag::Bag bag_new;

    string bag_new_path;
    string bag_read_pcd_path;
    string bag_read_img_path;
    string lidar_topic;
    string image_topic;

    nh.getParam("bag_new_path", bag_new_path);
    nh.getParam("bag_read_pcd_path", bag_read_pcd_path);
    nh.getParam("bag_read_img_path", bag_read_img_path);
    nh.getParam("lidar_topic", lidar_topic);
    nh.getParam("image_topic", image_topic);

    cout << "bag_new_path:" << bag_new_path << endl;
    cout << "bag_read_pcd_path:" << bag_read_pcd_path << endl;
    cout << "bag_read_img_path:" << bag_read_img_path << endl;
    cout << "lidar_topic:" << lidar_topic << endl;
    cout << "image_topic:" << image_topic << endl;

    bag_new.open(bag_new_path, rosbag::BagMode::Write);
    bag_read_pcd.open(bag_read_pcd_path);
    bag_read_img.open(bag_read_img_path);

    vector<string> topics_pcd;
    topics_pcd.emplace_back(lidar_topic);
    rosbag::View view_pcd(bag_read_pcd, rosbag::TopicQuery(topics_pcd));
    for (const rosbag::MessageInstance &m : view_pcd) {
        if (lidar_topic == m.getTopic()) {
            sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
            bag_new.write(m.getTopic(), ros::Time::now(), *msg);      // 注意时间戳，否则同时写入两个数据，读取时无数据显示
        }
    }

    vector<string> topics_img;
    topics_img.emplace_back(image_topic);
    rosbag::View view_img(bag_read_img, rosbag::TopicQuery(topics_img));
    for (const rosbag::MessageInstance &m : view_img) {
        if (image_topic == m.getTopic()) {
            sensor_msgs::Image::ConstPtr msg = m.instantiate<sensor_msgs::Image>();
            bag_new.write(m.getTopic(), ros::Time::now(), *msg);
        }
    }

    bag_new.close();
    bag_read_pcd.close();
    bag_read_img.close();

    return 0;
}