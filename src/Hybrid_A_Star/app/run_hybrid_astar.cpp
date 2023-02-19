#include "hybrid_a_star/hybrid_a_star_flow.h"

#include "3rd/backward.hpp"

#include <ros/ros.h>

namespace backward
{
    backward::SignalHandling sh;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "run_hybrid_astar");
    ros::NodeHandle node_handle("~");

    HybridAStarFlow kinodynamic_astar_flow(node_handle);

    ros::Rate rate(10);

    while (ros::ok())
    {
        kinodynamic_astar_flow.Run();

        ros::spinOnce();
        rate.sleep();
    }

    ros::shutdown();
    return 0;
}