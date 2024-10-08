#ifndef MODE_TAKEOFF_H
#define MODE_TAKEOFF_H

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <tucan_msgs/msg/mode.hpp>
#include <tucan_msgs/msg/mode_status.hpp>
#include <tucan_msgs/msg/ar_marker.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>

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
	float altitude_; // Takeoff altitude in meters (positive up)
	float takeoff_tolerance = 0.2; // Takeoff tolerance in meters 

	bool initiated_takeoff = false;		// Flag to check if takeoff has been initiated
	bool takeoff_finished = false;		// Flag to check if takeoff has finishe

	int desired_ar_id = -1;

	float ar_distance_sq;

	float takeoff_x_;				// Takeoff position in x direction
	float takeoff_y_;				// Takeoff position in y direction
	float takeoff_z_ = 0.0;				// Takeoff position in z direction
	float takeoff_yaw_ = 0.0;			// Takeoff yaw angle

	bool busy_ = false;

	bool vehicle_odom_received_ = false;
	bool vehicle_status_received_ = false;
	float last_ar_time_tolerance = 3.5;   // s, how long to use the last AR marker position after it has been lost	

	VehicleOdometry vehicle_odom_;
	VehicleStatus vehicle_status_;

	mode_status mode_status_;

	uint8_t own_mode_id_ = Mode::TAKEOFF; // Takeoff mode ID is 9, DON'T CHANGE
	
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Clock::SharedPtr clock;

	rclcpp::Subscription<Mode>::SharedPtr mission_state_subscriber;
	rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_odom_subscriber_;
	rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_subscriber_;
	rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr desired_altitude_subscriber_;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr ar_marker_id_subscriber_;
	rclcpp::Subscription<ARMarker>::SharedPtr ar_detector_subscriber_;

	rclcpp::Publisher<ModeStatus>::SharedPtr mode_status_publisher_;
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;

	void timer_callback();
	void publish_offboard_position_mode();
	void publish_trajectory_setpoint();
	void publish_mode_status();
	void process_ar_id_msg(const std_msgs::msg::Int32::SharedPtr msg);
	void process_ar_msg(const ARMarker::SharedPtr msg);
	void mission_state_callback(const Mode& msg);
	void vehicle_odom_callback(const VehicleOdometry& msg);
	void vehicle_status_callback(const VehicleStatus& msg);
	void takeoff();
	void desired_altitude_callback(const std_msgs::msg::Float32::SharedPtr msg);
};

#endif