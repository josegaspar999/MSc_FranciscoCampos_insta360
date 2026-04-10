#include <ros/ros.h>
#include "loop_closure/loop_closure_node.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "loop_closure_node");
    ros::NodeHandle nh;

    LoopClosureNode node(nh);

    ros::spin();
    return 0;
}