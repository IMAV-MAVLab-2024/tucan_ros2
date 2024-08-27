#ifndef MODE_LINE_FOLLOWER_H
#define MODE_LINE_FOLLOWER_H

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include <rclcpp/rclcpp.hpp>

#include <tucan_msgs/msg/line_follower.hpp>
#include <tucan_msgs/msg/ar_marker.hpp>
#include <tucan_msgs/msg/mode.hpp>
#include <tucan_msgs/msg/mode_status.hpp>


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
	uint8_t own_mode_id_ = 2;

	const float forward_vel = 1.0; 	// Forward velocity in m/s
	float lateral_vel;			   	// Lateral velocity in m/s 
	float yaw_reference;			// Yaw reference in radians

	const float K_lateral = 1.0;
	const float K_yaw = 0.1;

	const float ar_tolerance = 25.0; // Tolerance in pixels for AR marker detection. Exit condition.

	uint8_t last_ar_id;
	float line_angle;
	float ar_radius;
	
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr setpoint_publisher_;
	rclcpp::Publisher<ModeStatus>::SharedPtr mode_status_publisher_;

	rclcpp::Subscription<LineFollower>::SharedPtr line_detector_subscriber_;
	rclcpp::Subscription<ARMarker>::SharedPtr ar_detector_subscriber_;
	rclcpp::Subscription<Mode>::SharedPtr md_state_subscriber_;

	void publish_setpoint();
	void publish_mode_status();
	void timer_callback(void);
	
	void process_line_msg(const LineFollower::SharedPtr);
	void process_ar_msg(const ARMarker::SharedPtr);
	void process_state_msg(const Mode::SharedPtr);

	void activate_node();
	void deactivate_node();
};

#endif