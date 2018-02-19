#include "base_state.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace umigv;

using umigv::encoder_odometry::Vector2;
using umigv::encoder_odometry::Pose2;
using umigv::encoder_odometry::Twist2;
using umigv::encoder_odometry::BaseState;

Vector2 Vector2::from_polar(const f64 angle, const f64 magnitude) noexcept {
    return { magnitude * std::cos(angle), magnitude * std::sin(angle) };
}

Vector2& Vector2::operator+=(const Vector2 rhs) noexcept {
    x += rhs.x;
    y += rhs.y;

    return *this;
}

Vector2 operator+(const Vector2 lhs, const Vector2 rhs) noexcept {
    return { lhs.x + rhs.x, lhs.y + rhs.y };
}

static geometry_msgs::Quaternion
quaternion_from_yaw(const f64 yaw) noexcept {
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, yaw);

    return tf2::toMsg(orientation);
}

geometry_msgs::Pose Pose2::to_message() const noexcept {
    geometry_msgs::Pose message;

    message.position.x = position.x;
    message.position.y = position.y;

    message.orientation = quaternion_from_yaw(yaw);

    return message;
}

geometry_msgs::Twist Twist2::to_message() const noexcept {
    geometry_msgs::Twist message;

    message.linear.x = linear.x;
    message.linear.y = linear.y;

    message.angular.z = yaw_rate;

    return message;
}

// calculate change in position given current angle, wheel travel arclength,
// and change in angle
// https://en.wikipedia.org/wiki/Law_of_cosines#Version_suited_to_small_angles
static Vector2 calculate_dr(const f64 theta, const f64 dxbar,
                            const f64 dtheta) noexcept {
    f64 magnitude;

    if (dtheta != 0.0) {
        magnitude = 2.0 * std::abs(dxbar / dtheta * std::sin(dtheta / 2.0));
    } else {
        magnitude = std::abs(dxbar);
    }

    const f64 angle = theta + dtheta / 2.0;

    return Vector2::from_polar(angle, magnitude);
}

// calculate current velocity given wheel travel arclength, change in time, and
// current yaw
static Vector2 calculate_v(const f64 dxbar, const f64 dt,
                           const f64 next_theta) noexcept {
    const f64 magnitude = dxbar / dt;
    const f64 angle = next_theta;

    return { magnitude * std::cos(angle), magnitude * std::sin(angle) };
}

BaseState::BaseState(const f64 track) noexcept : track_{ track } { }

// $$ \Delta \theta = \frac{\Delta x_1 - \Delta x_0}{L} $$
// $$ \Delta \bar{x} = \frac{\Delta x_0 + \Delta x_1}{2} $$
// $$ || \Delta \vec{r} \, || = 2 \left| \frac{\Delta \bar{x}}{\Delta \theta} sin \frac{\Delta \theta}{2} \right| $$
// $$ \lim_{\Delta \theta \to 0} || \Delta \vec{r} \, || = |\bar{x}| $$
// $$ \angle \Delta \vec{r} = \theta + \frac{\Delta \theta}{2} $$
// $$ \Delta \vec{r} = || \Delta \vec{r} \, || \left[ cos \, \angle \Delta \vec{r}, sin \, \angle \Delta \vec{r} \, \right]^T $$
// $$ \vec{r'} = \vec{r} + \Delta \vec{r} $$
// $$ \theta' = \theta + \Delta \theta $$
// $$ || \vec{v} \, || = \frac{|\Delta \bar{x}|}{\Delta t} $$
// $$ \angle \vec{v} = \theta' $$
// $$ \omega = \frac{\Delta \theta}{\Delta t} $$
void BaseState::update(const f64 dx0, const f64 dx1,
                       const ros::Time t) noexcept {
    if (t <= timestamp_) {
        return;
    }

    const f64 dt = (t - timestamp_).toSec();

    const f64 dxbar = (dx0 + dx1) / 2.0;
    const f64 dtheta = (dx1 - dx0) / track_;

    timestamp_ = t;

    pose_.position += calculate_dr(pose_.yaw, dxbar, dtheta);
    pose_.yaw += dtheta;

    twist_.linear = calculate_v(dxbar, dt, pose_.yaw);
    twist_.yaw_rate = dtheta / dt;
}

const Pose2& BaseState::pose() const noexcept {
    return pose_;
}

const Twist2& BaseState::twist() const noexcept {
    return twist_;
}

ros::Time BaseState::timestamp() const noexcept {
    return timestamp_;
}
