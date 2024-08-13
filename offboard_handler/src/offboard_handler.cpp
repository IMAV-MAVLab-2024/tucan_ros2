/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
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
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */
#include <offboard_handler.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

using namespace px4_msgs::msg;

/**
 * @brief Offboard handler node: Makes sure offboard is kept on by publishing high frequency setpoints to the flight controller, even when other nodes
 * are not sending setpoints.
 */
OffboardHandler::OffboardHandler() :
		Node("offboard_handler"),
        offboard_control_mode_publisher_(this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10)),
		trajectory_setpoint_publisher_(this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10)),
		vehicle_command_publisher_(this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10)),
        command_subscriber_(this->create_subscription<TrajectorySetpoint>("/trajectory_setpoint", 10, std::bind(&OffboardHandler::command_callback, this, std::placeholders::_1))),
		control_mode_subscriber_(this->create_subscription<OffboardControlMode>("/control_mode", 10, std::bind(&OffboardHandler::control_mode_callback, this, std::placeholders::_1)))
{
	RCLCPP_INFO(this->get_logger(), "Starting Offboard Handler node");

    // Initialize setpoint as zero
    setpoint_.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    setpoint_.position = {0.0, 0.0, 0.0};
    setpoint_.velocity = {0.0, 0.0, 0.0};
    setpoint_.acceleration = {0.0, 0.0, 0.0};
    setpoint_.yaw = 0.0;
    setpoint_.yawspeed = 0.0;

    // Run offboard handler at 100 Hz
	timer_ = this->create_wall_timer(10ms, std::bind(&OffboardHandler::loop_once, this));
};

void OffboardHandler::loop_once()
{
    publish_offboard_control_mode();
    publish_trajectory_setpoint();
}

/**
 * @brief Callback for the trajectory setpoint subscription. Sets the message on the
 * @param msg   Trajectory setpoint message
 */
void OffboardHandler::command_callback(const TrajectorySetpoint::SharedPtr msg)
{
    setpoint_ = *msg;
}

void OffboardHandler::control_mode_callback(const OffboardControlMode::SharedPtr msg)
{
	control_mode_ = *msg;
}

/**
 * @brief Publish the offboard control mode.
 */
void OffboardHandler::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg = control_mode_;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);

	// For safety, set position control to true after publishing (e.g. hold position)
	control_mode_.position = true;
	control_mode_.velocity = false;
	control_mode_.acceleration = false;
	control_mode_.attitude = false;
	control_mode_.body_rate = false;
}

/**
 * @brief Publish a trajectory setpoint
 *        The setpoint is always the latest setpoint received by this node. The timestamp
 *       is the current time.
 */
void OffboardHandler::publish_trajectory_setpoint()
{
	TrajectorySetpoint msg{};
    msg = setpoint_;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);

	// For safety, set velocity and acceleration to 0 after publishing (e.g. hold position)
	setpoint_.velocity = {0.0, 0.0, 0.0};
	setpoint_.acceleration = {0.0, 0.0, 0.0};
}

int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardHandler>());

	rclcpp::shutdown();
	return 0;
}