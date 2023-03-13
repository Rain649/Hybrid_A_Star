#ifndef HYBRID_A_STAR_INIT_POSE_SUBSCRIBER_H
#define HYBRID_A_STAR_INIT_POSE_SUBSCRIBER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

class InitPoseSubscriber2D
{
public:
    InitPoseSubscriber2D(ros::NodeHandle &nh, const std::string &topic_name,
                         size_t buff_size);

    void ParseData(geometry_msgs::PoseWithCovarianceStampedPtr &data, bool &flag);

private:
    void MessageCallBack(const nav_msgs::Odometry &msg);

private:
    ros::Subscriber subscriber_;
    geometry_msgs::PoseWithCovarianceStampedPtr init_pose;

    bool start_flag;
};

#endif // HYBRID_A_STAR_INIT_POSE_SUBSCRIBER_H
