#include "hybrid_a_star/costmap_subscriber.h"

float detection_distance = 25.f;

CostMapSubscriber::CostMapSubscriber(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size)
{

    // callback flag
    odom_flag = false;
    target_flag = false;
    lidar_flag = false;
    tf_flag = false;

    // map
    map_rows = map_cols = 50;
    map_res = 1;
    // map_o_x = map_o_y = -map_rows * map_res / 2; // 在车辆坐标系下的地图原点坐标
    map_o_x = map_o_y = 0;
    image_map.create(map_rows, map_cols, CV_8UC1);

    MapPub = nh.advertise<nav_msgs::OccupancyGrid>("/cost_map", 1);
    map_msg.reset(new nav_msgs::OccupancyGrid());
    nh.param<std::string>("frame_id", map_msg->header.frame_id, "sg_map");
    map_msg->info.resolution = map_res;
    map_msg->info.width = map_cols;
    map_msg->info.height = map_rows;
    map_msg->info.origin.position.x = map_o_x;
    map_msg->info.origin.position.y = map_o_y;
    map_vec.resize(map_msg->info.width * map_msg->info.height);

    subscriber_ = nh.subscribe(topic_name, buff_size, &CostMapSubscriber::LidarCallback, this);

    // lidar
    Laser_Edge = abs(detection_distance);

    pass_z.setFilterFieldName("z");                        // 设置过滤时所需要点云类型的Z字段
    pass_z.setFilterLimits(-10, 1);                        // 设置在过滤字段的范围
    pass_x.setFilterFieldName("x");                        // 设置过滤时所需要点云类型的Z字段
    pass_x.setFilterLimits(0, 2 * Laser_Edge);             // 设置在过滤字段的范围
    pass_y.setFilterFieldName("y");                        // 设置过滤时所需要点云类型的Z字段
    pass_y.setFilterLimits(0, 2 * Laser_Edge);             // 设置在过滤字段的范围
    downSizeFilter.setLeafSize(map_res, map_res, map_res); // 分辨率

    localVertexCloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    localVertexCloud->header.frame_id = "localMap";
    LaserCloudSurround.reset(new pcl::PointCloud<pcl::PointXYZ>());
    LaserCloudSurroundFiltered.reset(new pcl::PointCloud<pcl::PointXYZ>());

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
    LaserCloudSurround->clear();

    pcl::PointCloud<pcl::PointXYZ> temp_cloud;

    pcl::fromROSMsg(*msg, temp_cloud);
    temp_cloud.header.frame_id = "vehicle_base_link";

    // 融合局部地图,坐标系转换：车辆坐标系to局部地图坐标系
    pcl::PointCloud<pcl::PointXYZ> pc_trans;

    if (listener.canTransform("/localMap", "vehicle_base_link", ros::Time(0)), "sorry")
    {
        // listener.waitForTransform("vehicle_base_link", "/localMap", ros::Time(0), ros::Duration(1));
        listener.lookupTransform("/localMap", "vehicle_base_link", ros::Time(0), transform_v2l);

        // tf::Vector3 v3 = transform_v2l.getOrigin();
        // printf("x: %f \n", v3.getX());
        // printf("x: %f \n", v3.getY());
        // printf("x: %f \n", v3.getZ());
        // transform_v2l.getRotation();

        geometry_msgs::PointStamped ip;
        geometry_msgs::PointStamped op;
        ip.header.frame_id = temp_cloud.header.frame_id;
        for (auto i : temp_cloud)
        {
            ip.point.x = i.x;
            ip.point.y = i.y;
            ip.point.z = i.z;
            listener.transformPoint("/localMap", ip, op);

            LaserCloudSurround->push_back({static_cast<float>(op.point.x),static_cast<float>(op.point.y),static_cast<float>(op.point.z)});
        }
    }
    *LaserCloudSurround += *localVertexCloud;

    for (auto &i : *LaserCloudSurround)
    {
        i.x += detection_distance;
        i.y += detection_distance;
    }
    LaserCloudSurround->header.frame_id = "sg_map";

    // ROS_INFO("LaserCloudSurround.size() = %zu", LaserCloudSurround->size());

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
        pcl::PointXYZ pointSub = LaserCloudSurround->points[i];

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

}
