#ifndef HYBRID_A_STAR_GOAL_POSE_SUBSCRIBER_H
#define HYBRID_A_STAR_GOAL_POSE_SUBSCRIBER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <deque>
#include <mutex>

class GoalPoseSubscriber2D
{
public:
    GoalPoseSubscriber2D(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size);

    void ParseData(std::deque<geometry_msgs::PoseStampedPtr> &pose_data_buff);

private:
    void MessageCallBack(const geometry_msgs::PoseStampedPtr &msg);

    // TF
    tf::TransformListener listener;
    tf::StampedTransform transform_v2l;

private:
    ros::Subscriber subscriber_;
    std::deque<geometry_msgs::PoseStampedPtr> goal_poses_;

    std::mutex buff_mutex_;
};

#endif // HYBRID_A_STAR_GOAL_POSE_SUBSCRIBER_H
