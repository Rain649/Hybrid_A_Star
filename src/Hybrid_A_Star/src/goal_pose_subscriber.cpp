#include "hybrid_a_star/goal_pose_subscriber.h"

extern float detection_distance;

GoalPoseSubscriber2D::GoalPoseSubscriber2D(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size)
{
    subscriber_ = nh.subscribe(
        topic_name, buff_size, &GoalPoseSubscriber2D::MessageCallBack, this);
    goal_pose = boost::make_shared<geometry_msgs::PoseStamped>();
}

void GoalPoseSubscriber2D::ParseData(geometry_msgs::PoseStampedPtr &data, bool &flag)
{
    *data = *goal_pose;
    flag = goal_flag;
    if (flag == true)
    {
        goal_flag = false;
    }
}

void GoalPoseSubscriber2D::MessageCallBack(const geometry_msgs::PoseStampedPtr &msg)
{
    goal_pose = msg;
    goal_pose->pose.position.x += detection_distance;
    goal_pose->pose.position.y += detection_distance;
    goal_flag = true;
}
