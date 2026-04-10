#ifndef __KEYFRAME_SELECTOR_HPP__
#define __KEYFRAME_SELECTOR_HPP__

#include <stdint.h>
#include <Eigen/Dense>

using Vector6d = Eigen::Matrix<double,6,1>;
using Matrix6d = Eigen::Matrix<double,6,6>;

enum {
    KEYFRAME_NOT_NEEDED = 0,
    TEMPORAL_POLICY_BLOCK,
    TRACKING_POLICY,
    SCENE_NOVELTY_POLICY,
    MOTION_POLICY,
};

class KeyframeSelector
{
private:
    // State
    Vector6d last_keyframe_pose_ = Vector6d::Zero();
    uint32_t frames_count_ = 0;
    // last keyframe time?

    // Motion policy parameters
    Matrix6d dist_weights_ = Matrix6d::Identity();
    double dist_threshold_ = 0.5;

    // Temporal policy parameters
    uint32_t minimum_frames_ = 10;
    // Minimum time ?

    // Tracking + Scene novelty policy parameters
    uint32_t min_feature_count_ = 20;
    double new_features_ratio_ = 0.2;
    
public:
    KeyframeSelector() = default;
    ~KeyframeSelector() = default;

    void set_distance_weights(Matrix6d dist_weights) { dist_weights_ = dist_weights; };
    void set_dist_threshold(double dist_threshold) { dist_threshold_ = dist_threshold; };
    void set_minimum_frames(uint32_t minimum_frames) { minimum_frames_ = minimum_frames; };
    void set_min_feature_count(uint32_t min_feature_count) { min_feature_count_ = min_feature_count; };
    void set_new_features_ratio(double new_features_ratio) { new_features_ratio_ = new_features_ratio; };

    uint32_t need_keyframe(const Vector6d& current_pose, uint32_t tracked_features, uint32_t new_features)
    {
        frames_count_++;

        // Temporal policy
        if (frames_count_ < minimum_frames_) {
            return TEMPORAL_POLICY_BLOCK;
        }

        // Tracking policy
        if (tracked_features < min_feature_count_) {
            return TRACKING_POLICY;
        }

        // Scene novelty policy
        double ratio = (double)new_features / ((double)tracked_features + (double)new_features);
        if (ratio > new_features_ratio_) {
            return SCENE_NOVELTY_POLICY;
        }

        // Motion policy
        auto diff = current_pose - last_keyframe_pose_;
        double dist = diff.transpose() * dist_weights_ * diff;
        if (dist > dist_threshold_) {
            return MOTION_POLICY;
        }

        return KEYFRAME_NOT_NEEDED;
    };

    void update_last_keyframe(const Vector6d& pose)
    {
        last_keyframe_pose_ = pose;
        frames_count_ = 0;
    };
};

#endif /*__KEYFRAME_SELECTOR_HPP__*/