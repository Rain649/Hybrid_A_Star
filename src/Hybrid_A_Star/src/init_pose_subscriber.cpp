#include "hybrid_a_star/init_pose_subscriber.h"

extern float detection_distance;

InitPoseSubscriber2D::InitPoseSubscriber2D(ros::NodeHandle &nh,
                                           const std::string &topic_name,
                                           size_t buff_size)
{
    subscriber_ = nh.subscribe(topic_name, buff_size, &InitPoseSubscriber2D::MessageCallBack, this);
    init_pose = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();
    init_pose->header.frame_id = "sg_map";
}

void InitPoseSubscriber2D::ParseData(geometry_msgs::PoseWithCovarianceStampedPtr &data, bool &flag)
{
    *data = *init_pose;
    flag = start_flag;
    if (flag == true)
    {
        start_flag = false;
    }
}

void InitPoseSubscriber2D::MessageCallBack(const nav_msgs::Odometry &msg)
{
    init_pose->header.stamp = msg.header.stamp;
    init_pose->pose.pose = msg.pose.pose;
    init_pose->pose.pose.position.x += detection_distance;
    init_pose->pose.pose.position.y += detection_distance;
    start_flag = true;
}