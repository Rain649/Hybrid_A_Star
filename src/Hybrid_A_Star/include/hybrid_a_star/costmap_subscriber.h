#ifndef HYBRID_A_STAR_COSTMAP_SUBSCRIBER_H
#define HYBRID_A_STAR_COSTMAP_SUBSCRIBER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <mutex>
#include <thread>
#include <string>
class VehState
{
public:
    // 横坐标, 纵坐标, 航向角, 线速度, 角速度
    float x, y, yaw, u, w;
};

class CostMapSubscriber {
public:
    CostMapSubscriber(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size);
    void ParseData(nav_msgs::OccupancyGridPtr &data, bool &flag);

private:
    void MessageCallBack(const nav_msgs::OccupancyGridPtr &costmap_msg_ptr);
    void LidarCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

    // void OdomCallback(const nav_msgs::Odometry &msg);
    // void TargetCallback(const nav_msgs::Odometry &msg);

private:
    ros::Subscriber subscriber_, OdomSub, TargetSub, LidarSub;
    ros::Publisher MapPub;

    // callback flag
    bool odom_flag, target_flag, map_flag, tf_flag;

    // TF
    tf::TransformListener listener;
    tf::StampedTransform transform_v2l;
    std::mutex mtx_listener;

    // vehicle
    VehState current_state;

    // local map
    pcl::PointCloud<pcl::PointXYZ>::Ptr localVertexCloud;

    // msg
    nav_msgs::OccupancyGridPtr map_msg; // map

    // map
    std::vector<signed char> map_vec;
    cv::Mat image_map; // 二值化地图
    cv::Mat image_dis; // 距离地图
    int map_rows, map_cols; // 地图行数、列数
    float map_o_x, map_o_y; // 原点
    float map_res; // 分辨率
    int index_row, index_col;

    // // Dubins/Reeds-Shepp曲线
    // double start_state[3], start_state_veh[3];
    // double final_state[3], final_state_veh[3];

    // lidar
    float Laser_Edge; // 雷达范围
    pcl::PointCloud<pcl::PointXYZ>::Ptr LaserCloudSurround;
    pcl::PointCloud<pcl::PointXYZ>::Ptr LaserCloudSurroundFiltered;
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pcl::VoxelGrid<pcl::PointXYZ> downSizeFilter;
};

#endif //HYBRID_A_STAR_COSTMAP_SUBSCRIBER_H
