#include "twist_publisher.h"

#include <cmath>
#include <algorithm>
#include <iterator>
#include <tuple>
#include <utility>
#include <vector>

#include <umigv_utilities/ranges.hpp>

namespace umigv {
namespace encoder_odometry {
namespace detail {

static std::pair<boost::optional<f64>, boost::optional<f64>>
get_positions(const std::vector<std::string> &names,
              const std::vector<f64> &positions,
              const Frames &frames) {
    boost::optional<f64> left_position;
    boost::optional<f64> right_position;

    for (auto &&name_position_tuple : umigv::zip(names, positions)) {
        const std::string &name = std::get<0>(name_position_tuple);
        const f64 position = std::get<1>(name_position_tuple);

        if (name == frames.left) {
            left_position = position;

            if (right_position) {
                break;
            }
        } else if (name == frames.right) {
            right_position = position;

            if (left_position) {
                break;
            }
        }
    }

    return { left_position, right_position };
}

static boost::optional<State>
make_next_state(const sensor_msgs::JointState &joints, const Frames &frames) {
    boost::optional<f64> left_position;
    boost::optional<f64> right_position;

    std::tie(left_position, right_position) =
        get_positions(joints.name, joints.position, frames);

    if (!left_position || !right_position) {
        ROS_WARN_STREAM("JointState did not contain left or right position");
        return boost::none;
    }

    const State to_return{ joints.header.stamp, left_position.value(),
                           right_position.value() };

    return { to_return };
}

struct Twist2DStamped {
    ros::Time stamp;
    f64 linear_x_velocity;
    f64 angular_z_velocity;
};

static Twist2DStamped make_twist_2d(const boost::circular_buffer<State> &states,
                                    const BaseSpecifications &specs) {
    if (states.size() < 2) {
        throw std::invalid_argument{ "make_twist_2d" };
    }

    const std::tuple<f64, f64, ros::Duration> sums =
        umigv::reduce(
            umigv::map(
                umigv::pairs(states),
                [](auto &&pair) {
                    const State &previous = pair.first;
                    const State &current = pair.second;

                    return std::make_tuple(
                        current.left_rotation - previous.left_rotation,
                        current.right_rotation - previous.right_rotation,
                        current.stamp - previous.stamp
                    );
                }
            ), [](auto &&lhs, auto &&rhs) {
                return std::make_tuple(
                    std::get<0>(lhs) + std::get<0>(rhs),
                    std::get<1>(lhs) + std::get<1>(rhs),
                    std::get<2>(lhs) + std::get<2>(rhs)
                );
            }
        );

    const auto num_pairs = static_cast<f64>(states.size() - 1);

    const ros::Duration dt_bar{ std::get<2>(sums).toSec() / num_pairs };
    const f64 dtheta_l_bar = std::get<0>(sums) / num_pairs;
    const f64 dtheta_r_bar = std::get<1>(sums) / num_pairs;

    // + rotation -> + translation
    const f64 dp_l = dtheta_l_bar * specs.wheel_diameter / 2.0;

     // + rotation -> - translation
    const f64 dp_r = -dtheta_r_bar * specs.wheel_diameter / 2.0;

    const f64 v_l = dp_l / dt_bar.toSec();
    const f64 v_r = dp_r / dt_bar.toSec();

    Twist2DStamped twist;

    twist.stamp = states.back().stamp;
    twist.linear_x_velocity = (v_l + v_r) / 2.0;
    twist.angular_z_velocity = (v_r - v_l) / specs.base_track;

    return twist;
}

static void update_twist(geometry_msgs::TwistWithCovarianceStamped &twist,
                         const Twist2DStamped &update) {
    twist.header.stamp = update.stamp;

    twist.twist.twist.linear.x = update.linear_x_velocity;
    twist.twist.twist.linear.y = 0.0;
    twist.twist.twist.linear.z = 0.0;

    twist.twist.twist.angular.x = 0.0;
    twist.twist.twist.angular.y = 0.0;
    twist.twist.twist.angular.z = update.angular_z_velocity;
}

} // namespace detail

TwistPublisherBuilder&
TwistPublisherBuilder::with_wheel_diameter(const f64 diameter) {
    if (std::isinf(diameter) || std::isnan(diameter) || diameter <= 0.0) {
        throw std::invalid_argument{
            "TwistPublisherBuilder::with_wheel_diameter"
        };
    }

    diameter_ = diameter;

    return *this;
}

TwistPublisherBuilder& TwistPublisherBuilder::with_base_track(const f64 track) {
    if (std::isinf(track) || std::isnan(track) || track <= 0.0) {
        throw std::invalid_argument{ "TwistPublisherBuilder::with_base_track" };
    }

    track_ = track;

    return *this;
}

TwistPublisherBuilder& TwistPublisherBuilder::with_left_wheel_frame(std::string frame) {
    if (right_frame_ && right_frame_.value() == frame) {
        throw std::invalid_argument{
            "TwistPublisherBuilder::with_left_wheel_frame"
        };
    }

    left_frame_ = std::move(frame);

    return *this;
}

TwistPublisherBuilder&
TwistPublisherBuilder::with_right_wheel_frame(std::string frame) {
    if (left_frame_ && left_frame_.value() == frame) {
        throw std::invalid_argument{
            "TwistPublisherBuilder::with_left_wheel_frame"
        };
    }

    right_frame_ = std::move(frame);

    return *this;
}

TwistPublisherBuilder&
TwistPublisherBuilder::with_covariance(std::vector<f64> covariance) {
    if (covariance.size() != 36) {
        throw std::invalid_argument{ "TwistPublisherBuilder::with_covariance" };
    }

    twist_covariance_ = std::move(covariance);

    return *this;
}

TwistPublisherBuilder&
TwistPublisherBuilder::with_frame_id(std::string frame) {
    twist_frame_ = std::move(frame);

    return *this;
}

TwistPublisherBuilder& TwistPublisherBuilder::from_node(
    ros::NodeHandle &node, const std::string &topic, const u32 queue_size
) {
    using TwistT = geometry_msgs::TwistWithCovarianceStamped;

    publisher_ = node.advertise<TwistT>(topic, queue_size);

    return *this;
}

TwistPublisher TwistPublisherBuilder::build() {
    if (!diameter_ || !track_ || !left_frame_ || !right_frame_
        || !twist_covariance_ || !twist_frame_ || !publisher_) {
        throw std::logic_error{ "TwistPublisherBuilder::build" };
    }

    BaseSpecifications specs{ diameter_.value(), track_.value() };
    Frames frames{ std::move(left_frame_.value()),
                   std::move(right_frame_.value()) };

    geometry_msgs::TwistWithCovarianceStamped prototype;

    prototype.header.seq = 0;
    prototype.header.stamp = ros::Time{ 0 };
    prototype.header.frame_id = std::move(twist_frame_.value());

    std::copy(twist_covariance_.value().cbegin(),
              twist_covariance_.value().cend(),
              prototype.twist.covariance.begin());

    prototype.twist.twist.linear.x = 0.0;
    prototype.twist.twist.linear.y = 0.0;
    prototype.twist.twist.linear.z = 0.0;

    prototype.twist.twist.angular.x = 0.0;
    prototype.twist.twist.angular.y = 0.0;
    prototype.twist.twist.angular.z = 0.0;

    return { std::move(specs), std::move(frames), std::move(prototype),
             std::move(publisher_.value()) };
}

void TwistPublisher::update_joint_states(
    const sensor_msgs::JointState::ConstPtr &states_ptr
) {
    const auto maybe_next_state = detail::make_next_state(*states_ptr, frames_);

    if (!maybe_next_state) {
        return;
    }

    states_.push_back(maybe_next_state.value());

    if (states_.size() > 1) {
        const detail::Twist2DStamped next_twist =
            detail::make_twist_2d(states_, specs_);
    
        detail::update_twist(twist_, next_twist);
    }
}

void TwistPublisher::publish_twist(const ros::TimerEvent&) {
    publisher_.publish(twist_);
    ++twist_.header.seq;
}

ros::Subscriber TwistPublisher::make_subscriber(ros::NodeHandle &node,
                                                const std::string &topic,
                                                const u32 queue_size) {
    return node.subscribe<sensor_msgs::JointState>(
        topic, queue_size, &TwistPublisher::update_joint_states, this
    );
}

ros::Timer TwistPublisher::make_timer(const ros::NodeHandle &node,
                                      const ros::Rate rate) {
    return node.createTimer(rate, &TwistPublisher::publish_twist, this);
}

TwistPublisher::TwistPublisher(
    BaseSpecifications specs, Frames frames,
    geometry_msgs::TwistWithCovarianceStamped prototype,
    ros::Publisher publisher
) noexcept : specs_{ std::move(specs) }, frames_{ std::move(frames) },
             twist_{ std::move(prototype) },
             publisher_{ std::move(publisher) } {
    states_.set_capacity(16);
}

} // namespace encoder_odometry
} // namespace umigv
