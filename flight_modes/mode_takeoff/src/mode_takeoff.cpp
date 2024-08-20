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
#include <px4_msgs/srv/vehicle_command.hpp>
#include <tucan_msgs/msg/mode.hpp>
#include <tucan_msgs/msg/mode_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include <mode_takeoff.hpp>

using namespace std::chrono_literals;
using namespace tucan_msgs::msg;
using namespace px4_msgs::msg;

ModeTakeoff::ModeTakeoff() :
		Node("mode_takeoff"),
		mode_status_(MODE_INACTIVE),
		mission_state_subscriber(this->create_subscription<Mode>("/mission_state", 10, std::bind(&ModeTakeoff::mission_state_callback, this, std::placeholders::_1))),
		mode_status_publisher_{this->create_publisher<ModeStatus>("/mode_status", 10)},
		offboard_control_mode_publisher_{this->create_publisher<OffboardControlMode>("/offboard_control_mode", 10)},
		trajectory_setpoint_publisher_{this->create_publisher<TrajectorySetpoint>("/trajectory_setpoint", 10)}
{
	RCLCPP_INFO(this->get_logger(), "Starting Takeoff mode");

	timer_ = this->create_wall_timer(100ms, std::bind(&ModeTakeoff::timer_callback, this));
}

/**
 * @brief Timer callback to publish mode status. If mode is active, calls takeoff once and sets mode to finished.
 */
void ModeTakeoff::timer_callback()
{
	publish_mode_status();

	if (mode_status_ == MODE_ACTIVE)
	{
		takeoff();
		counter++;
		if (counter > 20)
		{
			mode_status_ = MODE_FINISHED;
			counter = 0;
		}
	}
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void ModeTakeoff::publish_offboard_position_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 1 meters.
 */
void ModeTakeoff::publish_trajectory_setpoint()
{
	float z_position = -1.0*altitude_;
	TrajectorySetpoint msg{};
	msg.position = {0.0, 0.0, z_position};
	//msg.yaw = 0; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

void ModeTakeoff::publish_mode_status()
{
	// Publish the current state of the mode
	ModeStatus msg{};
	Mode mode{};
	mode.mode_id = own_mode_id_;
	msg.mode = mode;
	msg.mode_status = mode_status_;
	if (mode_status_ == MODE_ACTIVE)
	{
		msg.busy = true;
	}
	else
	{
		msg.busy = false;
	}
}

void ModeTakeoff::mission_state_callback(const Mode::SharedPtr msg)
{
	if (msg->mode_id == own_mode_id_)
	{
		mode_status_ = MODE_ACTIVE;
	}
}

void ModeTakeoff::takeoff(){
	publish_offboard_position_mode();
	publish_trajectory_setpoint();
}

int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ModeTakeoff>());

	rclcpp::shutdown();
	return 0;
}
