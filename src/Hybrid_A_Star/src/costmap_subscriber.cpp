#include "hybrid_a_star/costmap_subscriber.h"

CostMapSubscriber::CostMapSubscriber(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size)
{

    // callback flag
    odom_flag = false;
    target_flag = false;
    lidar_flag = false;
    tf_flag = false;

    // map
    map_rows = 200;
    map_cols = 200;
    map_res = 0.2;
    map_o_x = -20; // 在车辆坐标系下的地图原点坐标
    map_o_y = -20;
    image_map.create(map_rows, map_cols, CV_8UC1);

    MapPub = nh.advertise<nav_msgs::OccupancyGrid>("/map_local", 1);
    map_msg.reset(new nav_msgs::OccupancyGrid());
    nh.param<std::string>("frame_id", map_msg->header.frame_id, "vehicle_base_link");
    map_msg->info.resolution = map_res;
    map_msg->info.width = map_cols;
    map_msg->info.height = map_rows;
    map_msg->info.origin.position.x = map_o_x;
    map_msg->info.origin.position.y = map_o_y;
    map_vec.resize(map_msg->info.width * map_msg->info.height);

    subscriber_ = nh.subscribe(topic_name, buff_size, &CostMapSubscriber::LidarCallback, this);

    // lidar
    Laser_Edge = 20;

    pass_z.setFilterFieldName("z");                        // 设置过滤时所需要点云类型的Z字段
    pass_z.setFilterLimits(-10, 1);                        // 设置在过滤字段的范围
    pass_x.setFilterFieldName("x");                        // 设置过滤时所需要点云类型的Z字段
    pass_x.setFilterLimits(-Laser_Edge, Laser_Edge);       // 设置在过滤字段的范围
    pass_y.setFilterFieldName("y");                        // 设置过滤时所需要点云类型的Z字段
    pass_y.setFilterLimits(-Laser_Edge, Laser_Edge);       // 设置在过滤字段的范围
    downSizeFilter.setLeafSize(map_res, map_res, map_res); // 分辨率

    localVertexCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    LaserCloudSurround.reset(new pcl::PointCloud<pcl::PointXYZI>());
    LaserCloudSurroundFiltered.reset(new pcl::PointCloud<pcl::PointXYZI>());

    start_state_veh[0] = 0;
    start_state_veh[1] = 0;
    start_state_veh[2] = 0;
}

void CostMapSubscriber::MessageCallBack(const nav_msgs::OccupancyGridPtr &costmap_msg_ptr)
{
    buff_mutex_.lock();
    deque_costmap_.emplace_back(costmap_msg_ptr);
    buff_mutex_.unlock();
}

void CostMapSubscriber::ParseData(std::deque<nav_msgs::OccupancyGridPtr> &deque_costmap_msg_ptr)
{
    buff_mutex_.lock();
    if (!deque_costmap_.empty())
    {
        deque_costmap_msg_ptr.insert(deque_costmap_msg_ptr.end(),
                                     deque_costmap_.begin(),
                                     deque_costmap_.end());
        deque_costmap_.clear();
    }
    buff_mutex_.unlock();
}

void CostMapSubscriber::tfReceive()
{
    while (ros::ok())
    {
        if (odom_flag)
        {
            try
            {
                listener.lookupTransform("vehicle_base_link", "localMap",
                                         ros::Time(0), transform_l2v);
                tf_flag = true;
            }
            catch (tf::TransformException &ex)
            {
                // ROS_ERROR("Error: %s", ex.what());
                tf_flag = false;
                continue;
            }
        }
        else
        {
            tf_flag = false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    printf("TF Receive Exit.\n");
    return;
}
void CostMapSubscriber::OdomCallback(const nav_msgs::Odometry &msg)
{
    current_state.x = msg.pose.pose.position.x;
    current_state.y = msg.pose.pose.position.y;
    current_state.u = msg.twist.twist.linear.x;
    current_state.w = msg.twist.twist.angular.z;
    current_state.yaw = tf::getYaw(msg.pose.pose.orientation);

    start_state[0] = msg.pose.pose.position.x;
    start_state[1] = msg.pose.pose.position.y;
    start_state[2] = tf::getYaw(msg.pose.pose.orientation);

    odom_flag = true;
}

void CostMapSubscriber::TargetCallback(const nav_msgs::Odometry &msg)
{
    final_state[0] = msg.pose.pose.position.x;
    final_state[1] = msg.pose.pose.position.y;
    final_state[2] = tf::getYaw(msg.pose.pose.orientation);

    target_flag = true;
}
void CostMapSubscriber::LidarCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    buff_mutex_.lock();
    LaserCloudSurround->clear();
    pcl::fromROSMsg(*msg, *LaserCloudSurround);

    // 融合局部地图,坐标系转换：局部地图坐标系to车辆坐标系
    pcl::PointCloud<pcl::PointXYZI> pc_trans;

    if (tf_flag)
        pcl_ros::transformPointCloud(*localVertexCloud, pc_trans, transform_l2v);
    *LaserCloudSurround = pc_trans + *LaserCloudSurround;

    pass_z.setInputCloud(LaserCloudSurround);         // 设置输入点云
    pass_z.filter(*LaserCloudSurroundFiltered);       // 执行滤波
    pass_x.setInputCloud(LaserCloudSurroundFiltered); // 设置输入点云
    pass_x.filter(*LaserCloudSurround);               // 执行滤波
    pass_y.setInputCloud(LaserCloudSurround);         // 设置输入点云
    pass_y.filter(*LaserCloudSurroundFiltered);       // 执行滤波

    downSizeFilter.setInputCloud(LaserCloudSurroundFiltered);
    downSizeFilter.filter(*LaserCloudSurround);

    // 生成地图
    for (int i = 0; i < map_rows; ++i)
    {
        for (int j = 0; j < map_cols; ++j)
            image_map.at<uchar>(i, j) = 255;
    }
    for (size_t i = 0; i < LaserCloudSurround->points.size(); ++i)
    {
        pcl::PointXYZI pointSub = LaserCloudSurround->points[i];

        index_row = map_rows - 1 - floor((pointSub.y - map_o_y) / map_res);
        index_col = floor((pointSub.x - map_o_x) / map_res);

        if (index_row >= 0 && index_row < map_rows && index_col >= 0 && index_col < map_cols)
            image_map.at<uchar>(index_row, index_col) = 0;
    }

    cv::distanceTransform(image_map, image_dis, CV_DIST_L2, 5, CV_32FC1);

    for (size_t i = 0; i < map_msg->info.height; ++i)
    {
        for (size_t j = 0; j < map_msg->info.width; ++j)
        {
            // 图像上下颠倒
            if (image_map.at<uchar>(i, j) == 0)
                map_vec[(map_msg->info.height - 1 - i) * map_msg->info.width + j] = 100;
            else if (image_map.at<uchar>(i, j) == 255)
                map_vec[(map_msg->info.height - 1 - i) * map_msg->info.width + j] = 0;
        }
    }

    map_msg->data = map_vec;
    map_msg->header.stamp = ros::Time::now();
    MapPub.publish(*map_msg);
    deque_costmap_.emplace_back(map_msg);

    buff_mutex_.unlock();

    // 获取雷达边界点云
    // GetLaserEdgePoint();

    // lidar_flag = true;
}
