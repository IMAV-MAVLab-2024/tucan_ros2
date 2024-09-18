#ifndef MODE_LINE_FOLLOWER_H
#define MODE_LINE_FOLLOWER_H

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <rclcpp/rclcpp.hpp>

#include <tucan_msgs/msg/line_follower.hpp>
#include <tucan_msgs/msg/ar_marker.hpp>
#include <tucan_msgs/msg/mode.hpp>
#include <tucan_msgs/msg/mode_status.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>

using namespace px4_msgs::msg;
using namespace tucan_msgs::msg;

enum mode_status {
	MODE_ERROR,				// Something went wrong
	MODE_INACTIVE,			// Mode is inactive
	MODE_ACTIVE,			// Mode is active
	MODE_FINISHED			// Mode has finished
};

class ModeLineFollower : public rclcpp::Node
{
public:
	ModeLineFollower();

private:
	mode_status mode_status_;

	// Line Follow mode ID is 2, DON'T CHANGE
	uint8_t own_mode_id_ = Mode::LINE_FOLLOWER;

	const float forward_vel = 1.0; 	// Forward velocity in m/s
	float lateral_vel;			   	// Lateral velocity in m/s 
	float yaw_reference;			// Yaw reference in radians
	float yaw_running_average;		// Running average of the yaw for filtering
	float alpha = 0.14;

	float desired_altitude = 1.5; // Takeoff altitude in meters (negative up)

	// Liming
	float x_picture;
	float y_picture;
	float x_offset_dir;
	float y_offset_dir;

	float lateral_offset;

	bool vehicle_odom_received_ = false;

	VehicleOdometry vehicle_odom_;

	float last_ar_time_tolerance = 3.5;   // s, how long to use the last AR marker position after it has been lost	
	float last_line_time_tolerance = 2.0;   // s, how long to use the last AR marker position after it has been lost	

	const float sideward_gain = 0.2; // m
	const float forward_gain = 0.13; // m

	const float ar_tolerance = 1.5; // Tolerance in meters for AR marker detection. Exit condition.
	const float ar_tolerance_sq = ar_tolerance * ar_tolerance;

	float ar_distance_sq;

	rclcpp::Clock::SharedPtr clock;

	int desired_ar_id = -1;

	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr setpoint_publisher_;
	rclcpp::Publisher<ModeStatus>::SharedPtr mode_status_publisher_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr line_follower_activation_publisher_;

	rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_odom_subscriber_;
	rclcpp::Subscription<LineFollower>::SharedPtr line_detector_subscriber_;
	rclcpp::Subscription<ARMarker>::SharedPtr ar_detector_subscriber_;
	rclcpp::Subscription<Mode>::SharedPtr md_state_subscriber_;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr ar_marker_id_subscriber_;
	rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr desired_altitude_subscriber_;

	void publish_setpoint();
	void publish_mode_status();
	
	void process_line_msg(const LineFollower::SharedPtr);
	void process_ar_msg(const ARMarker::SharedPtr);
	void process_state_msg(const Mode::SharedPtr);
	void process_ar_id_msg(const std_msgs::msg::Int32::SharedPtr);
	void process_altitude_msg(const std_msgs::msg::Float32::SharedPtr);


	void vehicle_odom_callback(const VehicleOdometry& msg);

	void activate_node();
	void deactivate_node();
};

#endif