#ifndef UMIGV_ENCODER_ODOMETRY_BASE_STATE_H
#define UMIGV_ENCODER_ODOMETRY_BASE_STATE_H

#include <umigv_utilities/types.hpp> // umigv::f64

#include <ros/time.h> // ros::Time
#include <geometry_msgs/Pose.h> // geometry_msgs::Pose
#include <geometry_msgs/Twist.h> // geometry_msgs::Twist

namespace umigv {
namespace encoder_odometry {

// represents a vector in R^2
struct Vector2 {
    f64 x = 0.0;
    f64 y = 0.0;

    // construct a Vector2 from polar coordinates, right handed from [1, 0]^T
    static Vector2 from_polar(f64 angle, f64 magnitude) noexcept;

    Vector2& operator+=(Vector2 rhs) noexcept;
};

Vector2 operator+(Vector2 lhs, Vector2 rhs) noexcept;

// represents a pose in 2D space
struct Pose2 {
    Vector2 position;
    f64 yaw = 0.0;

    // convert this pose into a 3D pose on a plane
    geometry_msgs::Pose to_message() const noexcept;
};

// represents twist in 2D space
struct Twist2 {
    Vector2 linear;
    f64 yaw_rate = 0.0;

    // convert this twist into 3D twist on a plane
    geometry_msgs::Twist to_message() const noexcept;
};

// represents the state of a robot base based on wheel encoder data
class BaseState {
public:
    BaseState(f64 track) noexcept;

    // update the state with changes in wheel position and a timestamp
    void update(f64 dx0, f64 dx1, ros::Time t) noexcept;

    // get the pose of the robot's base in 2D space
    const Pose2& pose() const noexcept;

    // get the twist of the robot's base in 2D space
    const Twist2& twist() const noexcept;

    // get the time of the most recent update to the robot's state
    ros::Time timestamp() const noexcept;

private:
    Pose2 pose_;
    Twist2 twist_;
    ros::Time timestamp_;
    f64 track_;
};

} // namespace encoder_odometry
} // namespace umigv

#endif
