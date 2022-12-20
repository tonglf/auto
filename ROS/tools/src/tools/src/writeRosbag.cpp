#include <iostream>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/message_instance.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "write_rosbag");
    ros::NodeHandle nh("~");

    rosbag::Bag bag_read;
    rosbag::Bag bag_write;

    string bag_read_file;
    string bag_write_file;

    string topic;
    int loop = 0;
    int cnt = 10;

    nh.getParam("bag_read_file", bag_read_file);
    nh.getParam("bag_write_file", bag_write_file);
    nh.getParam("topic", topic);
    nh.getParam("loop", loop);
    nh.getParam("cnt", cnt);

    cout << "bag_read_file:" << bag_read_file << endl;
    cout << "bag_write_file:" << bag_write_file << endl;
    cout << "topic:" << topic << endl;
    cout << "loop:" << loop << endl;
    cout << "cnt:" << cnt << endl;

    bag_write.open(bag_write_file, rosbag::BagMode::Write);
    bag_read.open(bag_read_file);

    vector<string> topics;
    topics.emplace_back(topic);
    rosbag::View view(bag_read, rosbag::TopicQuery(topics));
    int frame = 0;
    for (const rosbag::MessageInstance &m : view) {
        if (topic == m.getTopic()) {
            ++frame;
            if (frame < cnt)
                continue;
            sensor_msgs::Image::ConstPtr msg = m.instantiate<sensor_msgs::Image>();
            for (int i = 0; i < loop; i++)
            {
                usleep(100000);
                bag_write.write(m.getTopic(), ros::Time::now(), *msg);
            }
            break;
        }
    }

    bag_write.close();
    bag_read.close();

    return 0;
}