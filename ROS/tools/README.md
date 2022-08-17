# README



## tools

### showpath

purpose：发布轨迹 topic，在 rviz 中显示实时轨迹。

method：使用 ROS 自带的 `nav_msgs/Path` 消息类型，查看该消息类型的内容：

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

usage：

```bash
source ./devel/setup.bash
roslaunch tools showpath.launch
```

参考博客：[（九）ROS在rviz中实时显示轨迹（nav_msgs/Path消息的使用）](https://blog.csdn.net/ktigerhero3/article/details/70256437)

### readrosbag
todo
