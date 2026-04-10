#include "loop_closure/loop_closure_node.hpp"

LoopClosureNode::LoopClosureNode(ros::NodeHandle& nh)
: nh_(nh)
{
    pose_sub_ = nh_.subscribe("pose_topic", 10, &LoopClosureNode::poseCallback, this);
    camstate_sub_ = nh_.subscribe("camstate_topic", 10, &LoopClosureNode::camstateCallback, this);

    keyframe_pub_ = nh_.advertise<geometry_msgs::PoseArray>("keyframes", 10);
    trigger_pub_ = nh_.advertise<std_msgs::UInt32>("keyframe_trigger", 10);

    selector_.set_dist_threshold(nh.param<double>("dist_threshold", 0.5));
    selector_.set_minimum_frames(nh.param<int>("minimum_frames", 10));
    selector_.set_min_feature_count(nh.param<int>("min_feature_count", 20));
    selector_.set_new_features_ratio(nh.param<double>("new_features_ratio", 0.2));

    std::vector<double> data;
    nh.getParam("dist_weights", data);
    selector_.set_distance_weights(Eigen::Map<Matrix6d>(data.data()));

    ROS_INFO("LoopClosureNode initialized.");
}

void LoopClosureNode::poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    double roll, pitch, yaw;
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    current_pose_(0) = msg->pose.pose.position.x;
    current_pose_(1) = msg->pose.pose.position.y;
    current_pose_(2) = msg->pose.pose.position.z;
    current_pose_(3) = roll;
    current_pose_(4) = pitch;
    current_pose_(5) = yaw;

    ROS_INFO("RX pose");
}

void LoopClosureNode::camstateCallback(const msckf_mono::TrackInfo::ConstPtr& msg)
{
    keyframe_trigger_ = selector_.need_keyframe(current_pose_, msg->tracked_features, msg->new_features);
    if (keyframe_trigger_ != KEYFRAME_NOT_NEEDED && keyframe_trigger_ != TEMPORAL_POLICY_BLOCK) {
        ROS_INFO("New keyframe triggered.");
        selector_.update_last_keyframe(current_pose_);

        // Store pose
        keyframes_.push_back(current_pose_);

        // Publish all keyframes for visualization
        publishKeyframes();
    }

    publishTrigger();

    ROS_INFO("RX frame");
}

void LoopClosureNode::publishKeyframes()
{
    geometry_msgs::PoseArray msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";

    for (const auto& kf : keyframes_) {
        geometry_msgs::Pose p;
        p.position.x = kf(0);
        p.position.y = kf(1);
        p.position.z = kf(2);

        tf::Quaternion q;
        q.setRPY(kf(3), kf(4), kf(5));
        tf::quaternionTFToMsg(q, p.orientation);

        msg.poses.push_back(p);
    }

    keyframe_pub_.publish(msg);
}


void LoopClosureNode::publishTrigger()
{
    std_msgs::UInt32 msg;
    msg.data = keyframe_trigger_;
    trigger_pub_.publish(msg);
}