#include "callback_handler.h"

#include <algorithm> // std::find, std::copy
#include <limits> // std::numeric_limits
#include <utility> // std::move

using namespace umigv;
using umigv::encoder_odometry::CallbackHandler;

CallbackHandler::CallbackHandler(
    ros::Publisher publisher, std::string frame_id, std::string child_frame_id,
    std::string left_wheel_frame_id, std::string right_wheel_frame_id,
    std::vector<f64> pose_covariance, std::vector<f64> twist_covariance,
    const f64 diameter, const f64 track
) : mutex_{ }, base_state_{ track }, frame_id_{ std::move(frame_id) },
    child_frame_id_{ std::move(child_frame_id) },
    left_frame_id_{ std::move(left_wheel_frame_id) },
    right_frame_id_{ std::move(right_wheel_frame_id) },
    pose_covariance_{ std::move(pose_covariance) },
    twist_covariance_{ std::move(twist_covariance) }, wheel_state_{ },
    publisher_{ std::move(publisher) }, meters_per_rad_{ diameter / 2.0 } { }

struct WheelJointIndices {
    usize left;
    usize right;
};

static WheelJointIndices
find_joint_indices(const sensor_msgs::JointState &state,
                   const std::string &left_frame,
                   const std::string &right_frame) {
    usize left_index = std::numeric_limits<usize>::max();
    usize right_index = std::numeric_limits<usize>::max();

    {
    usize i = 0;
    for (const auto &name : state.name) {
        if (name == left_frame) {
            left_index = i;
        }

        if (name == right_frame) {
            right_index = i;
        }

        if (left_index != std::numeric_limits<usize>::max()
            and right_index != std::numeric_limits<usize>::max()) {
            break;
        }

        ++i;
    }
    }

    if (left_index == std::numeric_limits<usize>::max()
        or right_index == std::numeric_limits<usize>::max()) {
        throw std::runtime_error{ " find_joint_indices" };
    }

    return { left_index, right_index };
}

void CallbackHandler::update_state(
    const sensor_msgs::JointState::ConstPtr &next_state_ptr
) {
    const auto &next_state = *next_state_ptr;

    const WheelJointIndices indices =
        find_joint_indices(next_state, left_frame_id_, right_frame_id_);

    const WheelState next_wheel_state{ next_state.position[indices.left],
                                       next_state.position[indices.right] };

    std::lock_guard<std::mutex> guard{ mutex_ };

    const f64 dx0 =
        (next_wheel_state.left - wheel_state_.left) * meters_per_rad_;
    const f64 dx1 =
        (next_wheel_state.right - wheel_state_.right) * meters_per_rad_;

    base_state_.update(dx0, dx1, next_state.header.stamp);
}

void CallbackHandler::publish_state(const ros::TimerEvent&) const {
    nav_msgs::Odometry message;

    std::lock_guard<std::mutex> guard{ mutex_ };

    message.header.seq = seq_id_++;
    message.header.stamp = base_state_.timestamp();
    message.header.frame_id = frame_id_;

    message.child_frame_id = child_frame_id_;

    message.pose.pose = base_state_.pose().to_message();
    std::copy(pose_covariance_.begin(), pose_covariance_.end(),
              message.pose.covariance.begin());

    message.twist.twist = base_state_.twist().to_message();
    std::copy(twist_covariance_.begin(), twist_covariance_.end(),
              message.twist.covariance.begin());

    publisher_.publish(message);
}
