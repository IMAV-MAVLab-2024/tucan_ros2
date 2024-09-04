#ifndef OPTI_T_H
#define OPTI_T_H

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <px4_frame_transforms_lib/frame_transforms.h>

#include <chrono>
#include <iostream>
#include <string>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OptitrackTimefix : public rclcpp::Node
{
public:
	OptitrackTimefix();

private:
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr setpoint_publisher_;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr visual_odom_subscription_;

	void visual_odom_callback(void);
};

#endif