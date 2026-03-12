# pcl_detection2使用说明

## 使用方式

### 启动节点

命令行内：

```shell
source /path/to/pcl_detection2/shell/pcl_detecion.sh
```

或者自己的shell内部
. /path/to/pcl_detection2/shell/pcl_detection.sh

### 调用话题

使用示例：

代码部分：

```cpp
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <vector>

void msg_cb(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    static pcl::PointCloud<pcl::PointXY>::Ptr pc(new pcl::PointCloud<pcl::PointXY>);
    pcl::fromROSMsg(*msg, *pc);
    ROS_INFO("points_num:%zu", pc->points.size());
}
int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "test_pcl_msg_node");
    ros::NodeHandle nh;

    auto pcl_sub = nh.subscribe("/projected_accumulated_cloud", 1, msg_cb);

    ros::spin();

    return 0;
}
```

CMakeLists:

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(test_pcl_msg)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
  pcl_conversions
  pcl_ros
)

catkin_package(
  CATKIN_DEPENDS
  sensor_msgs
  roscpp
  pcl_conversions
  pcl_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/main.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBARIRIES}
)

add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  )

```

package.xml:

```xml
<?xml version="1.0"?>
<package format="2">
  <name>test_pcl_msg</name>
  <version>0.0.0</version>
  <description>The test_pcl_msg package</description>
  <maintainer email="jetson@todo.todo">jetson</maintainer>
  <license>TODO</license>
  <buildtool_depend>catkin</buildtool_depend>
  <depend>roscpp</depend>
  <depend>sensor_msgs</depend>
  <depend>pcl_conversions</depend>
  <depend>pcl_ros</depend>
</package>

```
