#include <iostream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

int main (int argc, char** argv) { 
    ros::init(argc, argv, "readImage");
    ros::NodeHandle nh("~");

    string cloud_path;
    nh.getParam("cloud_path", cloud_path);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(cloud_path, *cloud);
    cout << "加载点云" << cloud->points.size() << "个" << endl;

    ros::Publisher pcd_pub = nh.advertise<sensor_msgs::PointCloud2>("pcd", 1, false);
    sensor_msgs::PointCloud2 cloud2;

    pcl::toROSMsg(*cloud, cloud2);
    cloud2.header.frame_id = "map";

    while (1)
    {
        pcd_pub.publish(cloud2);
    }
    
    return 0;
}
