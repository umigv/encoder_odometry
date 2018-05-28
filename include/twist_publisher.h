#ifndef UMIGV_ENCODER_ODOMETRY_TWIST_PUBLISHER_H
#define UMIGV_ENCODER_ODOMETRY_TWIST_PUBLISHER_H

#include <stdexcept>
#include <string>

#include <boost/circular_buffer.hpp>
#include <boost/optional.hpp>

#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <umigv_utilities/types.hpp>

namespace umigv {
namespace encoder_odometry {

struct State { // positive value corresponds to ccw rotation
    ros::Time stamp{ 0 };
    f64 left_rotation = 0.0;
    f64 right_rotation = 0.0;
};

struct BaseSpecifications { // relevant dimensions of the base, in meters
    f64 wheel_diameter = 0.0;
    f64 base_track = 0.0;
};

struct Frames { // frame IDs for the left and right wheel
    std::string left;
    std::string right;
};

class TwistPublisher;

class TwistPublisherBuilder {
public:
    TwistPublisherBuilder& with_wheel_diameter(f64 diameter);

    TwistPublisherBuilder& with_base_track(f64 track);

    TwistPublisherBuilder& with_left_wheel_frame(std::string frame);

    TwistPublisherBuilder& with_right_wheel_frame(std::string frame);

    TwistPublisherBuilder& with_covariance(std::vector<f64> covariance);

    TwistPublisherBuilder& with_frame_id(std::string frame);

    TwistPublisherBuilder& from_node(ros::NodeHandle &node,
                                     const std::string &topic,
                                     u32 queue_size);

    TwistPublisher build();

private:
    boost::optional<f64> diameter_;
    boost::optional<f64> track_;
    boost::optional<std::string> left_frame_;
    boost::optional<std::string> right_frame_;
    boost::optional<std::vector<f64>> twist_covariance_;
    boost::optional<std::string> twist_frame_;
    boost::optional<ros::Publisher> publisher_;
};

class TwistPublisher {
public:
    friend TwistPublisherBuilder;

    void update_joint_states(
        const sensor_msgs::JointState::ConstPtr &states_ptr
    );

    void publish_twist(const ros::TimerEvent &event);

    ros::Subscriber make_subscriber(ros::NodeHandle &node,
                                    const std::string &topic, u32 queue_size);

    ros::Timer make_timer(const ros::NodeHandle &node, ros::Rate rate);

private:
    TwistPublisher(BaseSpecifications specs, Frames frames,
                   geometry_msgs::TwistWithCovarianceStamped prototype,
                   ros::Publisher publisher) noexcept;

    boost::circular_buffer<State> states_;
    BaseSpecifications specs_;
    Frames frames_;
    geometry_msgs::TwistWithCovarianceStamped twist_;
    ros::Publisher publisher_;
};

} // namespace encoder_odometry
} // namespace umigv

#endif
