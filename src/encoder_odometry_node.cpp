#include "callback_handler.h" // umigv::encoder_odometry::CallbackHandler

#include <nav_msgs/Odometry.h> // nav_msgs::Odometry
#include <ros/ros.h> // ros::init, ros::Publisher, ROS_FATAL_STREAM, ros::Rate
#include <sensor_msgs/JointState.h> // sensor_msgs::JointState
#include <umigv_utilities/rosparam.hpp> // umigv::get_parameter_fatal,
                                        // umigv::get_parameter_or
#include <umigv_utilities/utility.hpp>

#include <cstdlib> // std::exit, EXIT_FAILURE
#include <string> // std::string
#include <thread> // std::thread
#include <utility> // std::move
#include <vector> // std::vector

using umigv::get_parameter_fatal;
using umigv::get_parameter_or;
using umigv::encoder_odometry::CallbackHandler;
using umigv::blocking_shutdown();

int main(int argc, const char *argv[]) {
    ros::init(argc, argv, "encoder_odometry_node");
    ros::NodeHandle handle;
    ros::NodeHandle private_handle{ "~" };

    auto publisher = handle.advertise<nav_msgs::Odometry>("encoders/odom", 10);

    auto frame_id = get_parameter_or<std::string>(private_handle, "frame_id",
                                                  "odom");
    auto child_frame_id = get_parameter_or<std::string>(private_handle,
                                                        "child_frame_id",
                                                        "encoders");
    auto left_wheel_frame_id =
        get_parameter_or<std::string>(private_handle, "left_wheel_frame_id",
                                      "wheel0");
    auto right_wheel_frame_id =
        get_parameter_or<std::string>(private_handle, "right_wheel_frame_id",
                                      "wheel1");

    std::vector<f64> pose_covariance;

    try {
        pose_covariance =
            get_parameter_fatal<std::vector<f64>>(private_handle,
                                                  "pose_covariance");
    } catch (const std::runtime_error &e) {
        ROS_FATAL_STREAM("unable to fetch ~pose_covariance");
        blocking_shutdown();
    }

    if (pose_covariance.size() != 36) {
        ROS_FATAL_STREAM("~pose_covariance must have length 36");
        blocking_shutdown();
    }

    std::vector<f64> twist_covariance;

    try {
        twist_covariance =
            get_parameter_fatal<std::vector<f64>>(private_handle,
                                                  "twist_covariance");
    } catch (const std::runtime_error &e) {
        ROS_FATAL_STREAM("unable to fetch ~twist_covariance");
        blocking_shutdown();
    }

    if (twist_covariance.size() != 36) {
        ROS_FATAL_STREAM("~twist_covariance must have length 36");
        blocking_shutdown();
    }

    f64 wheel_diameter;

    try {
        wheel_diameter = get_parameter_fatal<f64>(private_handle, "diameter");
    } catch (const std::runtime_error &e) {
        ROS_FATAL_STREAM("unable to fetch ~diameter");
        blocking_shutdown();
    }

    if (wheel_diameter <= 0.0) {
        ROS_FATAL_STREAM("~diameter must be a nonnegative number");
        blocking_shutdown();
    }

    f64 track;

    try {
        track = get_parameter_fatal<f64>(private_handle, "track");
    } catch (const std::runtime_error &e) {
        ROS_FATAL_STREAM("unable to fetch ~track");
        blocking_shutdown();
    }

    if (track <= 0.0) {
        ROS_FATAL_STREAM("~track must be a nonnegative number");
        blocking_shutdown();
    }

    const ros::Rate rate = get_parameter_or<f64>(private_handle, "rate", 20.0);

    CallbackHandler handler{ std::move(publisher), std::move(frame_id),
                             std::move(child_frame_id),
                             std::move(left_wheel_frame_id),
                             std::move(right_wheel_frame_id),
                             std::move(pose_covariance),
                             std::move(twist_covariance), wheel_diameter,
                             track };
    auto subscription =
        handle.subscribe<sensor_msgs::JointState>(
            "joint_states", 10, &CallbackHandler::update_state, &handler
        );

    auto timer = handle.createTimer(rate, &CallbackHandler::publish_states,
                                    &handler);

    ros::AsyncSpinner spinner{ std::thread::hardware_concurrency() };
    spinner.start();
    ros::waitForShutdown();
}
