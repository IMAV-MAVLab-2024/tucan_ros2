#ifndef MODE_LINE_FOLLOWER_H
#define MODE_LINE_FOLLOWER_H

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include <rclcpp/rclcpp.hpp>

#include <tucan_msgs/msg/line_follower.hpp>
#include <tucan_msgs/msg/ar_marker.hpp>
#include <tucan_msgs/msg/mode.hpp>
#include <tucan_msgs/msg/mode_status.hpp>

#include <std_msgs/msg/Bool.hpp>

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

	// Liming
	float x_picture;
	float y_picture;
	float z_global;
	float x_global;
	float y_global;
	

	const float K_lateral = 1.0;
	const float K_yaw = 0.1;

	const float ar_tolerance = 1; // Tolerance in meters for AR marker detection. Exit condition.
	const float ar_tolerance_sq = ar_tolerance * ar_tolerance;

	int last_ar_id;
	float line_angle;
	float ar_radius;

	int desired_ar_id = -1;
	
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr setpoint_publisher_;
	rclcpp::Publisher<ModeStatus>::SharedPtr mode_status_publisher_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr line_follower_activation_publisher_;

	rclcpp::Subscription<LineFollower>::SharedPtr line_detector_subscriber_;
	rclcpp::Subscription<ARMarker>::SharedPtr ar_detector_subscriber_;
	rclcpp::Subscription<Mode>::SharedPtr md_state_subscriber_;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr ar_marker_id_subscriber_;

	void publish_setpoint();
	void publish_mode_status();
	void timer_callback(void);
	
	void process_line_msg(const LineFollower::SharedPtr);
	void process_ar_msg(const ARMarker::SharedPtr);
	void process_state_msg(const Mode::SharedPtr);
	void process_ar_id_msg(const std_msgs::msg::Int32::SharedPtr);

	void activate_node();
	void deactivate_node();
};

#endif