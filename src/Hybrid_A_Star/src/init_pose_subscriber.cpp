#include "hybrid_a_star/init_pose_subscriber.h"

extern float detection_distance;

InitPoseSubscriber2D::InitPoseSubscriber2D(ros::NodeHandle &nh,
                                           const std::string &topic_name,
                                           size_t buff_size)
{
    subscriber_ = nh.subscribe(
        topic_name, buff_size, &InitPoseSubscriber2D::MessageCallBack, this);
}

// void InitPoseSubscriber2D::MessageCallBack(
//     const geometry_msgs::PoseWithCovarianceStampedPtr &init_pose_ptr)
// {
//     buff_mutex_.lock();
//     init_poses_.emplace_back(init_pose_ptr);
//     buff_mutex_.unlock();
// }

void InitPoseSubscriber2D::ParseData(
    std::deque<geometry_msgs::PoseWithCovarianceStampedPtr> &pose_data_buff)
{
    buff_mutex_.lock();
    if (!init_poses_.empty())
    {
        pose_data_buff.insert(pose_data_buff.end(), init_poses_.begin(), init_poses_.end());
        init_poses_.clear();
    }
    buff_mutex_.unlock();
}

void InitPoseSubscriber2D::MessageCallBack(const nav_msgs::Odometry &msg)
{
    geometry_msgs::PoseWithCovarianceStamped init_pose;
    init_pose.header.frame_id = "sg_map";
    init_pose.header.stamp = msg.header.stamp;
    init_pose.pose.pose = msg.pose.pose;
    init_pose.pose.pose.position.x += detection_distance;
    init_pose.pose.pose.position.y += detection_distance;
    geometry_msgs::PoseWithCovarianceStampedPtr init_pose_ptr = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>(init_pose);

    buff_mutex_.lock();
    init_poses_.emplace_back(init_pose_ptr);
    buff_mutex_.unlock();
}