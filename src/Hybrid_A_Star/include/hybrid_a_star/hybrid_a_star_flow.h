#ifndef HYBRID_A_STAR_HYBRID_A_STAR_FLOW_H
#define HYBRID_A_STAR_HYBRID_A_STAR_FLOW_H

#include "hybrid_a_star.h"
#include "costmap_subscriber.h"
#include "init_pose_subscriber.h"
#include "goal_pose_subscriber.h"
#include "std_msgs/Bool.h"

#include <ros/ros.h>

class HybridAStarFlow
{
public:
    HybridAStarFlow() = default;

    explicit HybridAStarFlow(ros::NodeHandle &nh);

    void Run();


private:
    void ReadData();

    void intersectionHandler(const std_msgs::Bool msg);

    void PublishPath(const VectorVec3d &path);

    void PublishSearchedTree(const VectorVec4d &searched_tree);

    void PublishVehiclePath(const VectorVec3d &path, double width,
                            double length, unsigned int vehicle_interval);

    void Reset_kinodynamic_astar_searcher();

    inline void Init_kinodynamic_astar_searcher();

private:
    std::shared_ptr<HybridAStar> kinodynamic_astar_searcher_ptr_;
    std::shared_ptr<CostMapSubscriber> costmap_sub_ptr_;
    std::shared_ptr<InitPoseSubscriber2D> init_pose_sub_ptr_;
    std::shared_ptr<GoalPoseSubscriber2D> goal_pose_sub_ptr_;

    ros::Subscriber subIntersection;

    ros::Publisher path_pub_;
    ros::Publisher searched_tree_pub_;
    ros::Publisher vehicle_path_pub_;

    geometry_msgs::PoseWithCovarianceStampedPtr current_init_pose_ptr_;
    geometry_msgs::PoseStampedPtr current_goal_pose_ptr_;
    nav_msgs::OccupancyGridPtr current_costmap_ptr_;

    ros::Time timestamp_;

    bool has_map_, has_goal_, has_start_, intersectionVerified, has_reset_;


    double steering_angle;
    int steering_angle_discrete_num;
    double wheel_base;
    double segment_length;
    int segment_length_discrete_num;
    double steering_penalty;
    double steering_change_penalty;
    double reversing_penalty;
    double shot_distance;
};

#endif // HYBRID_A_STAR_HYBRID_A_STAR_FLOW_H
