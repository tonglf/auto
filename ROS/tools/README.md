# README



## tools

### showpath

**purpose**：发布轨迹 topic，在 rviz 中显示实时轨迹。

**method**：使用 ROS 自带的 `nav_msgs/Path` 消息类型，查看该消息类型的内容：

```bash
~/tools$ rosmsg info nav_msgs/Path 
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/PoseStamped[] poses
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w

```

**usage**：

```bash
source ./devel/setup.bash
roslaunch tools showpath.launch
```

**note**：参考博客：[（九）ROS在rviz中实时显示轨迹（nav_msgs/Path消息的使用）](https://blog.csdn.net/ktigerhero3/article/details/70256437)



### readrosbag
**purpose**：读取 rosbag 中的点云数据，经处理后发布出来，并在 rviz 中显示。

**usage**：

1. 在 ymal 文件中写入相应的 rosbag 路径以及 topic；
2. 启动 launch 节点

```bash
roslaunch tools readrosbag
```



### mergerosbag

**purpose**：将一个 rosbag 中的 lidar 数据与另一个 rosbag 中的 Image 数据写入新的 rosbag 中。

**usage**：

	1. 在 ymal 文件中写入相应的 rosbag 路径以及 topic；
	1. 启动 launch 节点

```bash
roslaunch tools mergerosbag
```

**note**：在写入 rosbag 时，需注意 ros time，时间戳不能乱，否则写入数据完成后，数据不可被读取。
