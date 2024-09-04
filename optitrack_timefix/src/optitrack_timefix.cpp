#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <optitrack_timefix.hpp>

#include <iostream>
#include <string>

using namespace px4_msgs::msg;

OptitrackTimefix::OptitrackTimefix(std::string px4_namespace) :
		Node("optitrack_timefix")
{
	RCLCPP_INFO(this->get_logger(), "Starting optitrack timefix");

	visual_odom_subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/raw/fmu/in/vehicle_visual_odometry", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void ModeLineFollower::publish_offboard_velocity_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = true;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void ModeLineFollower::publish_velocity_setpoint()
{
	TrajectorySetpoint msg{};
	msg.velocity = {0.3, 0.0, 0.0};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

void ModeLineFollower::timer_callback(void){
	
	static uint8_t num_of_steps = 0;

	// offboard_control_mode needs to be paired with trajectory_setpoint
	publish_offboard_velocity_mode();
	publish_velocity_setpoint();
}

int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ModeLineFollower>("/fmu/"));

	rclcpp::shutdown();
	return 0;
}
