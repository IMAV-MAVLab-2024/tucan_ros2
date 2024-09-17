/****************************************************************************
 *
 * Copyright 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples * 
 * @author Beniamino Pozzan <beniamino.pozzan@gmail.com>
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include <tucan_msgs/msg/line_follower.hpp>
#include <tucan_msgs/msg/ar_marker.hpp>
#include <tucan_msgs/msg/mode.hpp>
#include <tucan_msgs/msg/mode_status.hpp>

#include <rclcpp/rclcpp.hpp>
#include <mode_line_follower.hpp>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace tucan_msgs::msg;

using std::placeholders::_1;

ModeLineFollower::ModeLineFollower() :
		Node("mode_line_follower"),
		mode_status_{MODE_INACTIVE},
		lateral_vel{0.0},
		yaw_reference{0.0},
		setpoint_publisher_{this->create_publisher<TrajectorySetpoint>("/trajectory_setpoint", 10)},
		mode_status_publisher_{this->create_publisher<ModeStatus>("/mode_status", 10)},
		line_follower_activation_publisher_{this->create_publisher<std_msgs::msg::Bool>("/cv_line_detector/enable", 5)},
		line_detector_subscriber_{this->create_subscription<LineFollower>("/cv_line_detection", 10, std::bind(&ModeLineFollower::process_line_msg, this, _1))},
		ar_detector_subscriber_{this->create_subscription<ARMarker>("/cv_aruco_detection", 10, std::bind(&ModeLineFollower::process_ar_msg, this, _1))},
		md_state_subscriber_{this->create_subscription<Mode>("/active_mode_id", 10, std::bind(&ModeLineFollower::process_state_msg, this, _1))},
		ar_marker_id_subscriber_{this->create_subscription<std_msgs::msg::Int32>("mode_line_follower/desired_id", 5, std::bind(&ModeLineFollower::process_ar_id_msg, this, _1))},
		desired_altitude_subscriber_{this->create_subscription<std_msgs::msg::Float32>("mode_line_follower/desired_altitude", 5, std::bind(&ModeLineFollower::process_altitude_msg, this, _1))}
{
	rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

	vehicle_odom_subscriber_ = this->create_subscription<VehicleOdometry>("fmu/out/vehicle_odometry", qos, std::bind(&ModeLineFollower::vehicle_odom_callback, this, std::placeholders::_1));
	
	clock = this->get_clock();
	RCLCPP_INFO(this->get_logger(), "Starting Line follower mode");
}

/**
 * @brief Publish a velocity setpoint
 *        Sends the set forward velocity and lateral velocity along with a yaw reference to the controller.
 */
void ModeLineFollower::publish_setpoint()
{
	//RCLCPP_INFO(this->get_logger(), "Publishing line setpoint");
	TrajectorySetpoint msg{};
	// msg.velocity = {x_picture/1000, y_picture/1000, 0.0};

	float x_forward_dir = cos(yaw_reference);
	float y_forward_dir = sin(yaw_reference);

	float x_desired = 0.0;
	float y_desired = 0.0;

	if (lateral_offset > 50 || lateral_offset < -50){ // towards the right
		x_desired = vehicle_odom_.position[0] + x_offset_dir * sideward_gain + forward_gain * x_forward_dir;
		y_desired = vehicle_odom_.position[1] + y_offset_dir * sideward_gain + forward_gain * y_forward_dir;
	}else{
		x_desired = vehicle_odom_.position[0] + forward_gain * x_forward_dir;
		y_desired = vehicle_odom_.position[1] + forward_gain * y_forward_dir;
	}

	msg.position = {x_desired, y_desired, -desired_altitude};
	msg.yaw = yaw_reference; // relative?
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish the mode status message
 *        Publishes the mode ID, mode status and busy flag to the mission director.
 * 		  After setting mode_status to MODE_FINISHED, call this function again to signal to the mission
 * 		  director that the mode has finished.
 */
void ModeLineFollower::publish_mode_status()
{
	// Publish the current state of the mode
	ModeStatus msg{};
	Mode mode{};
	mode.mode_id = own_mode_id_;
	msg.mode = mode;
	msg.mode_status = mode_status_;
	msg.busy = false;
	mode_status_publisher_->publish(msg);
}

/**
 * @brief Process the state message. If the published state is this state, set mode to active.
 */
void ModeLineFollower::process_state_msg(const Mode::SharedPtr msg)
{
	if (msg->mode_id == own_mode_id_)
	{
		if (mode_status_ == MODE_INACTIVE){
			activate_node();
		}else{
			// activate the cv line detector
			std_msgs::msg::Bool msg;
			msg.data = true;
			line_follower_activation_publisher_->publish(msg);
		}
	}else{
		if (mode_status_ == MODE_ACTIVE){
			deactivate_node();
		}
		mode_status_ = MODE_INACTIVE;
	}
}

/**
 * @brief Process the line message. Set the lateral velocity and yaw reference based on the line detection.
 */
void ModeLineFollower::process_line_msg(const LineFollower::SharedPtr msg)
{
	if (mode_status_== MODE_ACTIVE)
	{
		if (msg->detected)
		{
			yaw_reference = msg->yaw;
			x_picture = msg->x_picture;
			y_picture = msg->y_picture;
			x_offset_dir = msg->x_offset_dir;
			y_offset_dir = msg->y_offset_dir;
			lateral_offset = msg->lateral_offset;
			publish_setpoint();
			publish_mode_status();
		}else{
			// Get the current time
			rclcpp::Time now = clock->now();

			// Let's assume `time_difference` is a rclcpp::Duration object
			rclcpp::Duration time_difference = now - msg->last_detection_timestamp;  // 1.5 seconds as an example

			// Convert the result to seconds (nanoseconds are represented as int64_t)
			double time_difference_seconds = time_difference.seconds();

			if (time_difference_seconds < last_line_time_tolerance){
				yaw_reference = msg->yaw;
				x_picture = msg->x_picture;
				y_picture = msg->y_picture;
				x_offset_dir = msg->x_offset_dir;
				y_offset_dir = msg->y_offset_dir;
				lateral_offset = msg->lateral_offset;
				publish_setpoint();
				publish_mode_status();
			}
		}
	}
}

/**
 * @brief Process the AR marker message. If the marker is in the center of the image, deactivate the node.
 */
void ModeLineFollower::process_ar_msg(const ARMarker::SharedPtr msg)
{
	if (mode_status_ == MODE_ACTIVE)
	{
		if (msg->detected){
			if (desired_ar_id == -1 || desired_ar_id == msg->id){
				float x_dist = msg->x_global - vehicle_odom_.position[0];
				float y_dist = msg->y_global - vehicle_odom_.position[1];
				ar_distance_sq = x_dist*x_dist + y_dist*y_dist;
				RCLCPP_INFO(this->get_logger(), "AR distance squared %f", ar_distance_sq);
				if(ar_distance_sq < ar_tolerance_sq) // equals 0 if there is no ar marker -> >0.0001 to avoid exiting
				{
					deactivate_node();
				}
			}
		}else if (msg->id != 0)
		{
			// Get the current time
			rclcpp::Time now = clock->now();

			// Let's assume `time_difference` is a rclcpp::Duration object
			rclcpp::Duration time_difference = now - msg->last_detection_timestamp;  // 1.5 seconds as an example

			// Convert the result to seconds (nanoseconds are represented as int64_t)
			double time_difference_seconds = time_difference.seconds();

			if (time_difference_seconds < last_ar_time_tolerance){
				if (desired_ar_id == -1 || desired_ar_id == msg->id){
					float x_dist = msg->x_global - vehicle_odom_.position[0];
					float y_dist = msg->y_global - vehicle_odom_.position[1];
					ar_distance_sq = x_dist*x_dist + y_dist*y_dist;
					RCLCPP_INFO(this->get_logger(), "AR distance squared (no detection) %f", ar_distance_sq);
					if(ar_distance_sq < ar_tolerance_sq) // equals 0 if there is no ar marker -> >0.0001 to avoid exiting
					{
						deactivate_node();
					}
				}
			}
		}
	}
}

void ModeLineFollower::vehicle_odom_callback(const VehicleOdometry& msg)
{
	vehicle_odom_ = msg;
	vehicle_odom_received_ = true;
}

void ModeLineFollower::process_altitude_msg(const std_msgs::msg::Float32::SharedPtr msg)
{
	desired_altitude = msg->data;
}

/**
 * @brief Activate the node
 * 	  Set the mode status to MODE_ACTIVE and publish a log info message.
 */
void ModeLineFollower::activate_node()
{
	// activate the cv line detector
	std_msgs::msg::Bool msg;
	msg.data = true;
	line_follower_activation_publisher_->publish(msg);
	mode_status_ = MODE_ACTIVE;
	RCLCPP_INFO(this->get_logger(), "Mode line follower activated");
}

void ModeLineFollower::process_ar_id_msg(const std_msgs::msg::Int32::SharedPtr msg)
{
	desired_ar_id = msg->data;
}

/**
 * @brief Deactivate the node
 * 	  Set the mode status to MODE_FINISHED, publish a log info message, and signal the mission director.
 */
void ModeLineFollower::deactivate_node()
{
	// activate the cv line detector
	std_msgs::msg::Bool msg;
	msg.data = false;
	line_follower_activation_publisher_->publish(msg);

	desired_ar_id = -1;

	mode_status_ = MODE_FINISHED;

	RCLCPP_INFO(this->get_logger(), "Mode line follower finished");
	publish_mode_status();
}


// -----------------------------------------------------------------------------
int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ModeLineFollower>());

	rclcpp::shutdown();
	return 0;
}
