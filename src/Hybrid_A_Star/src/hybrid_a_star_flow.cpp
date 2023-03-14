#include "hybrid_a_star/hybrid_a_star_flow.h"

#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

float detection_distance = 25.f;

double Mod2Pi(const double &x)
{
    double v = fmod(x, 2 * M_PI);

    if (v < -M_PI)
    {
        v += 2.0 * M_PI;
    }
    else if (v > M_PI)
    {
        v -= 2.0 * M_PI;
    }

    return v;
}

HybridAStarFlow::HybridAStarFlow(ros::NodeHandle &nh)
{
    steering_angle = nh.param("planner/steering_angle", 10);
    steering_angle_discrete_num = nh.param("planner/steering_angle_discrete_num", 1);
    wheel_base = nh.param("planner/wheel_base", 1.0);
    segment_length = nh.param("planner/segment_length", 1.6);
    segment_length_discrete_num = nh.param("planner/segment_length_discrete_num", 8);
    steering_penalty = nh.param("planner/steering_penalty", 1.05);
    steering_change_penalty = nh.param("planner/steering_change_penalty", 1.5);
    reversing_penalty = nh.param("planner/reversing_penalty", 2.0);
    shot_distance = nh.param("planner/shot_distance", 3.0);

    kinodynamic_astar_searcher_ptr_ = std::make_shared<HybridAStar>(
        steering_angle, steering_angle_discrete_num, segment_length, segment_length_discrete_num, wheel_base, steering_penalty, reversing_penalty, steering_change_penalty, shot_distance);
    costmap_sub_ptr_ = std::make_shared<CostMapSubscriber>(nh, "/lidarCloudProcess/cloud_Combined", 1);
    init_pose_sub_ptr_ = std::make_shared<InitPoseSubscriber2D>(nh, "/navigation/intersectionOdom", 1);
    goal_pose_sub_ptr_ = std::make_shared<GoalPoseSubscriber2D>(nh, "/navigation/targetPoint", 1);
    subIntersection = nh.subscribe<std_msgs::Bool>("/intersectionDetection/intersectionVerified", 1, &HybridAStarFlow::intersectionHandler, this);

    path_pub_ = nh.advertise<nav_msgs::Path>("searched_path", 1);
    searched_tree_pub_ = nh.advertise<visualization_msgs::Marker>("searched_tree", 1);
    vehicle_path_pub_ = nh.advertise<visualization_msgs::MarkerArray>("vehicle_path", 1);

    has_goal_ = false;
    has_start_ = false;
    has_map_ = false;
    has_reset_ = true;

    current_init_pose_ptr_ = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();
    current_goal_pose_ptr_ = boost::make_shared<geometry_msgs::PoseStamped>();
    current_costmap_ptr_ = boost::make_shared<nav_msgs::OccupancyGrid>();
}

void HybridAStarFlow::intersectionHandler(const std_msgs::Bool msg)
{
    intersectionVerified = msg.data;
}

void HybridAStarFlow::Reset_kinodynamic_astar_searcher()
{
    kinodynamic_astar_searcher_ptr_.reset(new HybridAStar(steering_angle, steering_angle_discrete_num, segment_length, segment_length_discrete_num, wheel_base, steering_penalty, reversing_penalty, steering_change_penalty, shot_distance));
}

void HybridAStarFlow::Init_kinodynamic_astar_searcher()
{
    kinodynamic_astar_searcher_ptr_->Init(
        current_costmap_ptr_->info.origin.position.x,
        1.0 * current_costmap_ptr_->info.width * current_costmap_ptr_->info.resolution + current_costmap_ptr_->info.origin.position.x,
        current_costmap_ptr_->info.origin.position.y,
        1.0 * current_costmap_ptr_->info.height * current_costmap_ptr_->info.resolution + current_costmap_ptr_->info.origin.position.y,
        current_costmap_ptr_->info.resolution);
}

void HybridAStarFlow::Run()
{
    ReadData();

    if (!intersectionVerified || !has_start_ || !has_goal_ || !has_map_)
    {
        if (!has_reset_)
        {
            Reset_kinodynamic_astar_searcher();
            has_reset_ = true;
        }
        return;
    }
    else
    {
        ROS_ERROR("Have enough flags to run");
        const double map_resolution = 0.2;

        Init_kinodynamic_astar_searcher();

        ROS_ERROR("11");
        unsigned int map_w = std::floor(current_costmap_ptr_->info.width / map_resolution);
        unsigned int map_h = std::floor(current_costmap_ptr_->info.height / map_resolution);
        for (unsigned int w = 0; w < map_w; ++w)
        {
            for (unsigned int h = 0; h < map_h; ++h)
            {
                auto x = static_cast<unsigned int>((w + 0.5) * map_resolution / current_costmap_ptr_->info.resolution);
                auto y = static_cast<unsigned int>((h + 0.5) * map_resolution / current_costmap_ptr_->info.resolution);

                if (current_costmap_ptr_->data[y * current_costmap_ptr_->info.width + x])
                {
                    kinodynamic_astar_searcher_ptr_->SetObstacle(w, h);
                }
            }
        }
        has_start_ = false;
        has_goal_ = false;
        has_map_ = false;
        ROS_ERROR("22");

        double start_yaw = tf::getYaw(current_init_pose_ptr_->pose.pose.orientation);
        double goal_yaw = tf::getYaw(current_goal_pose_ptr_->pose.orientation);

        Vec3d start_state = Vec3d(
            current_init_pose_ptr_->pose.pose.position.x,
            current_init_pose_ptr_->pose.pose.position.y,
            start_yaw);

        ROS_ERROR("x = %f, y = %f", current_init_pose_ptr_->pose.pose.position.x - detection_distance, current_init_pose_ptr_->pose.pose.position.y - detection_distance);

        Vec3d goal_state = Vec3d(
            current_goal_pose_ptr_->pose.position.x,
            current_goal_pose_ptr_->pose.position.y,
            goal_yaw);

        ROS_ERROR("33");

        if (kinodynamic_astar_searcher_ptr_->Search(start_state, goal_state))
        {
            ROS_ERROR("Find Way");
            auto path = kinodynamic_astar_searcher_ptr_->GetPath();
            PublishPath(path);
            PublishVehiclePath(path, 4.0, 2.0, 5u);
            // ROS_ERROR("10-2 problem here");
            PublishSearchedTree(kinodynamic_astar_searcher_ptr_->GetSearchedTree());
        }

        ROS_ERROR("44");
        // debug
        //        std::cout << "visited nodes: " << kinodynamic_astar_searcher_ptr_->GetVisitedNodesNumber() << std::endl;
        kinodynamic_astar_searcher_ptr_->Reset();
        ROS_ERROR("55");
        has_reset_ = false;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void HybridAStarFlow::ReadData()
{
    init_pose_sub_ptr_->ParseData(current_init_pose_ptr_, has_start_);
    goal_pose_sub_ptr_->ParseData(current_goal_pose_ptr_, has_goal_);
    costmap_sub_ptr_->ParseData(current_costmap_ptr_, has_map_);
}

void HybridAStarFlow::PublishPath(const VectorVec3d &path)
{
    nav_msgs::Path nav_path;

    geometry_msgs::PoseStamped pose_stamped;
    for (const auto &pose : path)
    {
        pose_stamped.header.frame_id = "sg_map";
        pose_stamped.pose.position.x = pose.x();
        pose_stamped.pose.position.y = pose.y();
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(pose.z());

        nav_path.poses.emplace_back(pose_stamped);
    }

    nav_path.header.frame_id = "sg_map";
    nav_path.header.stamp = timestamp_;

    path_pub_.publish(nav_path);
}

void HybridAStarFlow::PublishVehiclePath(const VectorVec3d &path, double width,
                                         double length, unsigned int vehicle_interval = 5u)
{
    visualization_msgs::MarkerArray vehicle_array;

    for (unsigned int i = 0; i < path.size(); i += vehicle_interval)
    {
        visualization_msgs::Marker vehicle;

        if (i == 0)
        {
            vehicle.action = 3;
        }

        vehicle.header.frame_id = "sg_map";
        vehicle.header.stamp = ros::Time::now();
        vehicle.type = visualization_msgs::Marker::CUBE;
        vehicle.id = static_cast<int>(i / vehicle_interval);
        vehicle.scale.x = width;
        vehicle.scale.y = length;
        vehicle.scale.z = 0.01;
        vehicle.color.a = 0.1;

        vehicle.color.r = 1.0;
        vehicle.color.b = 0.0;
        vehicle.color.g = 0.0;

        vehicle.pose.position.x = path[i].x();
        vehicle.pose.position.y = path[i].y();
        vehicle.pose.position.z = 0.0;

        vehicle.pose.orientation = tf::createQuaternionMsgFromYaw(path[i].z());
        vehicle_array.markers.emplace_back(vehicle);
    }

    vehicle_path_pub_.publish(vehicle_array);
}

void HybridAStarFlow::PublishSearchedTree(const VectorVec4d &searched_tree)
{
    visualization_msgs::Marker tree_list;
    tree_list.header.frame_id = "sg_map";
    tree_list.header.stamp = ros::Time::now();
    tree_list.type = visualization_msgs::Marker::LINE_LIST;
    tree_list.action = visualization_msgs::Marker::ADD;
    tree_list.ns = "searched_tree";
    tree_list.scale.x = 0.02;

    tree_list.color.a = 1.0;
    tree_list.color.r = 0;
    tree_list.color.g = 0;
    tree_list.color.b = 0;

    tree_list.pose.orientation.w = 1.0;
    tree_list.pose.orientation.x = 0.0;
    tree_list.pose.orientation.y = 0.0;
    tree_list.pose.orientation.z = 0.0;

    geometry_msgs::Point point;
    for (const auto &i : searched_tree)
    {
        point.x = i.x();
        point.y = i.y();
        point.z = 0.0;
        tree_list.points.emplace_back(point);

        point.x = i.z();
        point.y = i.w();
        point.z = 0.0;
        tree_list.points.emplace_back(point);
    }

    searched_tree_pub_.publish(tree_list);
}
