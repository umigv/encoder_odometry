## Synopsis

`encoder_odometry` contains `encoder_odometry_node`, a node to output geometry_msgs/TwistWithCovarianceStamped data from subscribed JointState messages.

## Motivation

`encoder_odometry_node` allows for wheel encoder based localization.

## Installation

Clone this repository into your `catkin` workspace, then run `catkin_make_isolated --install --pkg encoder_odometry`.

## Contributors

`encoder_odometry` is authored and maintained by Gregory Meyer.
