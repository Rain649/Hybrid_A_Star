#ifndef HYBRID_A_STAR_GOAL_POSE_SUBSCRIBER_H
#define HYBRID_A_STAR_GOAL_POSE_SUBSCRIBER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

class GoalPoseSubscriber2D
{
public:
    GoalPoseSubscriber2D(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size);

    void ParseData(geometry_msgs::PoseStampedPtr &data, bool &flag);

private:
    void MessageCallBack(const geometry_msgs::PoseStampedPtr &msg);

private:
    ros::Subscriber subscriber_;
    geometry_msgs::PoseStampedPtr goal_pose;

    bool goal_flag;
};

#endif // HYBRID_A_STAR_GOAL_POSE_SUBSCRIBER_H
