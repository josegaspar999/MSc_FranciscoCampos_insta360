#ifndef __ROS_INTERFACE_HPP__
#define __ROS_INTERFACE_HPP__

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/UInt32.h>
#include "msckf_mono/TrackInfo.h"
#include "loop_closure/keyframe_selector.hpp"
#include <tf/transform_datatypes.h>

class LoopClosureNode
{
public:
    LoopClosureNode(ros::NodeHandle& nh);

private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    ros::Subscriber camstate_sub_;
    ros::Publisher keyframe_pub_;
    ros::Publisher trigger_pub_;

    Vector6d current_pose_ = Vector6d::Zero();
    KeyframeSelector selector_;

    // Store all keyframe poses for publishing
    std::vector<Vector6d> keyframes_;
    uint32_t keyframe_trigger_;

    void poseCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void camstateCallback(const msckf_mono::TrackInfo::ConstPtr& msg);
    void publishKeyframes();
    void publishTrigger();
};

#endif /*__ROS_INTERFACE_HPP__*/