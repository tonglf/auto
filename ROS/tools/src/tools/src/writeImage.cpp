#include <iostream>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/message_instance.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "writeImage");
    ros::NodeHandle nh("~");

    bool compressed = false;
    string bag_path;
    string image_topic;
    string image_output_path;
    nh.getParam("compressed", compressed);
    nh.getParam("bag_path", bag_path);
    nh.getParam("image_topic", image_topic);
    nh.getParam("image_output_path", image_output_path);

    cout << "bag_path: " << bag_path << endl;
    cout << "image_topic: " << image_topic << endl;
    cout << "image_output_path: " << image_output_path << endl;

    rosbag::Bag bag;
    bag.open(bag_path);

    vector<string> topics;
    topics.emplace_back(image_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    int index = 0;
    for (const rosbag::MessageInstance &m : view) {
        if (image_topic == m.getTopic()) {
            cv_bridge::CvImagePtr cv_ptr;
            if (compressed) {
                sensor_msgs::CompressedImage::ConstPtr camera_msg = m.instantiate<sensor_msgs::CompressedImage>();
                 cv_ptr = cv_bridge::toCvCopy(camera_msg, sensor_msgs::image_encodings::BGR8);
            } else {
                sensor_msgs::Image::ConstPtr camera_msg = m.instantiate<sensor_msgs::Image>();
                cv_ptr = cv_bridge::toCvCopy(camera_msg, sensor_msgs::image_encodings::BGR8);
            }
            cv::Mat image = cv_ptr->image.clone();
            double time = cv_ptr->header.stamp.toSec();
            cv::imwrite(image_output_path + to_string(time) + ".jpg", image);
        }
    }
    bag.close();

    return 0;
}
