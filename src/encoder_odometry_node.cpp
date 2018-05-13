#include "twist_publisher.h"

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>
#include <umigv_utilities/types.hpp>
#include <umigv_utilities/rosparam.hpp>
#include <umigv_utilities/utility.hpp>

using namespace umigv::types;

struct Parameters {
    std::string frame_id;
    std::string left_wheel_frame;
    std::string right_wheel_frame;
    ros::Rate rate{ 0.0 };
    std::vector<f64> covariance_matrix; // 6x6
    f64 diameter;
    f64 track;
};

static Parameters get_parameters(ros::NodeHandle &node) {
    using namespace std::literals;
    using CovarianceT = std::vector<f64>;

    Parameters params;

    params.frame_id = umigv::get_parameter_or(node, "frame_id", "odom"s);
    params.left_wheel_frame =
        umigv::get_parameter_or(node, "left_wheel_frame", "wheel0"s);
    params.right_wheel_frame =
        umigv::get_parameter_or(node, "right_wheel_frame", "wheel1"s);
    params.rate = ros::Rate{ umigv::get_parameter_or(node, "rate", 20.0) };

    params.covariance_matrix =
        umigv::get_parameter_fatal<CovarianceT>(node, "pose_covariance");

    if (params.covariance_matrix.size() != 36) {
        throw std::logic_error{ "get_parameters" };
    }

    params.diameter = umigv::get_parameter_fatal<f64>(node, "diameter");

    if (params.diameter <= 0.0 || std::isnan(params.diameter)
        || std::isinf(params.diameter)) {
        throw std::logic_error{ "get_parameters" };
    }

    params.track = umigv::get_parameter_fatal<f64>(node, "track");

    if (params.track <= 0.0 || std::isnan(params.track)
        || std::isinf(params.track)) {
        throw std::logic_error{ "get_parameters" };
    }

    return params;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "encoder_odometry_node");

    ros::NodeHandle node;
    ros::NodeHandle private_node{ "~" };

    Parameters params = [&private_node] {
        try {
            return get_parameters(private_node);
        } catch (const umigv::ParameterNotFoundException &e) {
            ROS_FATAL_STREAM("unable to find parameter " << e.parameter());
            umigv::blocking_shutdown();
        } catch (const std::logic_error &e) {
            ROS_FATAL_STREAM("fetched an invalid parameter");
            umigv::blocking_shutdown();
        }
    }();

    auto publisher = umigv::encoder_odometry::TwistPublisherBuilder{ }
        .with_wheel_diameter(params.diameter)
        .with_base_track(params.track)
        .with_left_wheel_frame(std::move(params.left_wheel_frame))
        .with_right_wheel_frame(std::move(params.right_wheel_frame))
        .with_covariance(std::move(params.covariance_matrix))
        .from_node(node, "encoders/twist", 10)
        .build();

    const auto subscriber = 
        publisher.make_subscriber(node, "encoders/state", 10);
    const auto timer = publisher.make_timer(node, std::move(params.rate));

    ros::spin();
}
