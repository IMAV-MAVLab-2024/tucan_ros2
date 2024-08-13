#ifndef MODE_TAKEOFF_H
#define MODE_TAKEOFF_H

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace px4_msgs::msg;

class ModeTakeoff : public rclcpp::Node
{
public:
	ModeTakeoff();

private:
	float altitude_ = 1.0; // Takeoff altitude in meters (positive up)

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client_;

	void publish_offboard_position_mode();
	void publish_trajectory_setpoint();
	void takeoff();
};

#endif