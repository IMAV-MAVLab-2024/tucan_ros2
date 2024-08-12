#ifndef MODE_LINE_FOLLOWER_H
#define MODE_LINE_FOLLOWER_H

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <px4_frame_transforms_lib/frame_transforms.h>
#include <tucan_msgs/msg/line_follower.hpp>
#include <tucan_msgs/msg/ar_marker.hpp>
#include <tucan_msgs/msg/mode.hpp>

#include <chrono>
#include <iostream>
#include <string>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace geometry_msgs::msg;
using namespace std_msgs::msg;
using namespace tucan_msgs::msg;

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

	uint8_t own_mode_id = 2;

	bool active;
	float lateral_vel;
	float yaw_reference;

	const float forward_vel = 1.0;
	const float K_lateral = 1.0;
	const float K_yaw = 0.1;
	
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<Bool>::SharedPtr mode_finished_publisher_;

	rclcpp::Subscription<LineFollower>::SharedPtr line_detector_subscriber_;
	rclcpp::Subscription<ARMarker>::SharedPtr ar_detector_subscriber_;
	rclcpp::Subscription<Mode>::SharedPtr md_state_subscriber_;

	void publish_offboard_velocity_mode();
	void publish_velocity_setpoint();
	void publish_state_information();
	void timer_callback(void);
	
	void process_line_msg(const LineFollower::SharedPtr);
	void process_ar_msg(const ARMarker::SharedPtr) const;
	void process_state_msg(const Mode::SharedPtr);

	void activate_node();
};

#endif