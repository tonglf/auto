#include <iostream>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/message_instance.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "readImage");
    ros::NodeHandle nh("~");

    string bag_path;
    string image_topic;
    string image_output_path;
    nh.getParam("bag_path", bag_path);
    nh.getParam("image_topic", image_topic);
    nh.getParam("image_output_path", image_output_path);

    cout << "bag_path: " << bag_path << endl;
    cout << "image_topic: " << image_topic << endl;
    cout << "image_output_path: " << image_output_path << endl;

    ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("Image", 1, false);

    rosbag::Bag bag;
    bag.open(bag_path);

    vector<string> topics;
    topics.emplace_back(image_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    int index = 0;
    for (const rosbag::MessageInstance &m : view) {
        if (image_topic == m.getTopic()) {
            if (index % 5 == 0) {
                sensor_msgs::Image::ConstPtr camera_msg = m.instantiate<sensor_msgs::Image>();
                cv_bridge::CvImagePtr cv_ptr;
                cv_ptr = cv_bridge::toCvCopy(camera_msg, sensor_msgs::image_encodings::BGR8);
                cv::Mat image = cv_ptr->image.clone();
                cv::imwrite(image_output_path + to_string(index) + "_image.jpg", image);
                image_pub.publish(*camera_msg);
            }
            index++;
        }
    }
    bag.close();

    return 0;
}
