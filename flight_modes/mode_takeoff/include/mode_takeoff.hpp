#ifndef MODE_TAKEOFF_H
#define MODE_TAKEOFF_H

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <tucan_msgs/msg/mode.hpp>
#include <tucan_msgs/msg/mode_status.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace tucan_msgs::msg;
using namespace px4_msgs::msg;

enum mode_status {
	MODE_ERROR,				// Something went wrong
	MODE_INACTIVE,			// Mode is inactive
	MODE_ACTIVE,			// Mode is active
	MODE_FINISHED			// Mode has finished
};

class ModeTakeoff : public rclcpp::Node
{
public:
	ModeTakeoff();

private:
	float altitude_ = 1.0; // Takeoff altitude in meters (positive up)

	mode_status mode_status_;

	uint8_t own_mode_id_ = 9;
	
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Subscription<Mode>::SharedPtr mission_state_subscriber;
	rclcpp::Publisher<ModeStatus>::SharedPtr mode_status_publisher_;
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client_;

	void timer_callback();
	void publish_offboard_position_mode();
	void publish_trajectory_setpoint();
	void publish_mode_status();
	void mission_state_callback(const Mode::SharedPtr msg);
	void takeoff();
};

#endif