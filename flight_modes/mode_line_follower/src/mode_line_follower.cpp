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
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <std_msgs/msg/bool.hpp>

#include <tucan_msgs/msg/line_follower.hpp>
#include <tucan_msgs/msg/ar_marker.hpp>
#include <tucan_msgs/msg/mode.hpp>
#include <tucan_msgs/msg/mode_status.hpp>

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <mode_line_follower.hpp>

#include <chrono>
#include <iostream>
#include <string>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace geometry_msgs::msg;
using namespace std_msgs::msg;
using namespace tucan_msgs::msg;

using std::placeholders::_1;

ModeLineFollower::ModeLineFollower() :
		Node("mode_line_follower"),
		state_{State::init},
		mode_status_{MODE_INACTIVE},
		setpoint_publisher_{this->create_publisher<TrajectorySetpoint>("/trajectory_setpoint", 10)},
		mode_status_publisher_{this->create_publisher<ModeStatus>("mode_status", 10)},
		line_detector_subscriber_{this->create_subscription<LineFollower>("cv_line_detection", 10, std::bind(&ModeLineFollower::process_line_msg, this, _1))},
		ar_detector_subscriber_{this->create_subscription<ARMarker>("cv_ar_detection", 10, std::bind(&ModeLineFollower::process_ar_msg, this, _1))},
		md_state_subscriber_{this->create_subscription<Mode>("mission_state", 10, std::bind(&ModeLineFollower::process_state_msg, this, _1))}
{
	RCLCPP_INFO(this->get_logger(), "Starting Line follower mode");

	timer_ = this->create_wall_timer(100ms, std::bind(&ModeLineFollower::timer_callback, this));
}

void ModeLineFollower::timer_callback(void)
{
	
	publish_state_information();
}


/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void ModeLineFollower::publish_setpoint()
{
	TrajectorySetpoint msg{};
	msg.velocity = {forward_vel, lateral_vel, 0.0};
	msg.yaw = yaw_reference; // relative?
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

void ModeLineFollower::publish_state_information()
{
	// Publish the current state of the mode
	ModeStatus msg{};
	msg.mode = own_mode_id_;
	msg.mode_status = mode_status_;
	msg.busy = true;
}

void ModeLineFollower::process_state_msg(const Mode::SharedPtr msg)
{
	if (msg->mode_id == own_mode_id)
	{
		active = true;
	}
}

void ModeLineFollower::process_line_msg(const LineFollower::SharedPtr msg)
{
	lateral_vel = K_lateral * msg->avg_offset;
	yaw_reference = K_yaw * msg->angle;
}

void ModeLineFollower::process_ar_msg(const ARMarker::SharedPtr msg) const
{
	// If an AR marker is detected, hover above the marker and publish a finish message


	// If the AR marker is in the center of the image, deactivate node
	// TO DO: Tune this value
	if(msg->x*msg->x + msg->y*msg->y < 10)
	{
		deactivate_node();
	}
}

void ModeLineFollower::activate_node()
{
	active = true;
}

void ModeLineFollower::deactivate_node()
{
	Bool msg{};
	msg.data = true;
	mode_finished_publisher_->publish(msg);
}


// -----------------------------------------------------------------------------
int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ModeLineFollower>("/fmu/"));

	rclcpp::shutdown();
	return 0;
}
