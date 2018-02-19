#ifndef UMIGV_ENCODER_ODOMETRY_CALLBACK_HANDLER_H
#define UMIGV_ENCODER_ODOMETRY_CALLBACK_HANDLER_H

#include "base_state.h" // umigv::encoder_odometry::BaseState
#include "umigv_utilities/types.hpp" // umigv::f64

#include <nav_msgs/Odometry.h> // nav_msgs::Odometry
#include <ros/publisher.h> // ros::Publisher
#include <sensor_msgs/JointState.h> // sensor_msgs::JointState

#include <mutex> // std::mutex, std::lock_guard
#include <stdexcept> // std::runtime_error
#include <string> // std::string
#include <vector> // std::vector

namespace umigv {
namespace encoder_odometry {

// represents the rotation of wheels (in radians)
struct WheelState {
    f64 left = 0.0;
    f64 right = 0.0;
};

// asynchronous ROS wrappper for BaseState; calculates the current pose and
// twist of the robot on a plane in R^3
class CallbackHandler {
public:
    CallbackHandler(ros::Publisher publisher, std::string frame_id,
                    std::string child_frame_id, std::string left_wheel_frame_id,
                    std::string right_wheel_frame_id,
                    std::vector<f64> pose_covariance,
                    std::vector<f64> twist_covariance, f64 diameter, f64 track);

    // update the internal state
    // throws std::runtime_error if cannot find left_frame_id_ or
    // right_frame_id_ in received message
    void update_state(const sensor_msgs::JointState::ConstPtr &next_state_ptr);

    // publish the internal state as a nav_msgs/Odometry message
    void publish_state(const ros::TimerEvent &event) const;

private:
    mutable std::mutex mutex_; // lock whenever modifying internal state
    BaseState base_state_;
    const std::string frame_id_;
    const std::string child_frame_id_;
    const std::string left_frame_id_;
    const std::string right_frame_id_;
    std::vector<f64> pose_covariance_;
    std::vector<f64> twist_covariance_;
    WheelState wheel_state_;
    const ros::Publisher publisher_;
    const f64 meters_per_rad_;
    u32 seq_id_;
};

} // namespace encoder_odometry
} // namespace umigv

#endif
