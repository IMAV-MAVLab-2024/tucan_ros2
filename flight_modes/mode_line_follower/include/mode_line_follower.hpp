#ifndef MODE_LINE_FOLLOWER_H
#define MODE_LINE_FOLLOWER_H

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <px4_frame_transforms_lib/frame_transforms.h>

#include <chrono>
#include <iostream>
#include <string>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace geometry_msgs::msg;
using namespace std_msgs::msg;

class ModeLineFollower : public rclcpp::Node
{
public:
	ModeLineFollower(std::string px4_namespace);

private:
	enum class State{
		init,
		flying_forward,
		flying_backward
	} state_;

	uint8_t mode_id = 2;

	struct {
		float x;
		float y;
	} target, target_vel, zero;

	
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;

	rclcpp::Subscription<Point>::SharedPtr target_subscriber_;
	rclcpp::Subscription<UInt8>::SharedPtr md_state_subscriber_;

	void publish_offboard_velocity_mode();
	void publish_velocity_setpoint();
	void timer_callback(void);
	void process_msg(const Point::SharedPtr);
	void process_state_msg(const UInt8::SharedPtr) const;
};

#endif