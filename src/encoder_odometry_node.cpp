#include <ros/ros.h> // ros::init, ros::Publisher
#include <nav_msgs/Odometry.h> // nav_msgs::Odometry
#include <sensor_msgs/JointState.h> // sensor_msgs::JointState

#include <algorithm> // std::find
#include <iterator> // std::distance
#include <string> // std::string
#include <utility> // std::move, std::pair

class OdometryHandler {
    template <typename T>
    using IsPublisherT =
        std::enable_if_t<std::is_same<ros::Publisher, T>::value>;

public:
    template <typename T, typename = IsPublisherT>
    OdometryHandler(T &&publisher, const double radius, const double track)
        : publisher_{ publisher }, state_{ },
          meters_per_rad_{ radius }, track_{ track }
    { }

    void callback(const sensor_msgs::JointState::ConstPtr &state_ptr) {
        const auto &state = *state_ptr;


    }

private:
    std::pair<std::size_t, std::size_t> get_indices(
        const sensor_msgs::JointState &state
    ) const {
        const auto left_wheel_it = std::find(state.name.cbegin(),
                                             state.name.cend(),
                                             left_wheel_joint_);

        if (left_wheel_it == state.name.cend()) {
            throw "left wheel joint not found";
        }

        const auto left_wheel_index =
            static_cast<std::size_t>(std::distance(state.name.cbegin(),
                                                   left_wheel_it));

        if (left_wheel_index >= state.velocity.size()) {
            throw "no velocity for left wheel";
        }

        const auto right_wheel_it = std::find(state.name.cbegin(),
                                              state.name.cend(),
                                              right_wheel_joint_);

        if (right_wheel_it == state.name.cend()) {
            throw "right wheel joint not found";
        }

        const auto right_wheel_index =
            static_cast<std::size_t>(std::distance(state.name.cbegin(),
                                                   right_wheel_it));

        if (right_wheel_index >= state.velocity.size()) {
            throw "no velocity for right wheel";
        }

        return std::pair<std::size_t, std::size_t>{ left_wheel_index,
                                                    right_wheel_index };
    }

    ros::Publisher publisher_;
    nav_msgs::Odometry state_;
    std::string left_wheel_joint_;
    std::string right_wheel_joint_;
    double meters_per_rad_;
    double track_;
};

int main(int argc, const char *argv[]) {
    ros::init(argc, argv, "encoder_odometry_node");
    auto handle = ros::NodeHandle{ };
    auto private_handle = ros::NodeHandle{ "~" };

    auto publisher = handle.advertise<nav_msgs::Odometry>("encoders/odom", 10);

    auto handler = OdometryHandler{ std::move(publisher) };
    auto subscription =
        handle.subscribe<sensor_msgs::JointState>("joint_states", 10,
                                                  &OdometryHandler::callback,
                                                  &handler);


}
