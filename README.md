## Synopsis

`encoder_odometry` contains `encoder_odometry_node`, a node to output Odometry data from subscribed JointState messages and tf transforms.

## Code Example

    rosrun encoder_odometry encoder_odometry_node ~diameter:=0.3302 ~rate:=20 joint_states:=states encoders/odom:=odom

## Motivation

`encoder_odometry_node` allows for wheel encoder based odometry, which is a significant source of localization information.

## Installation

Clone this repository into your `catkin` workspace, then run `catkin_make`. Optionally, run `catkin_make install`.

## Contributors

`encoder_odometry` is authored and maintained by Gregory Meyer.
