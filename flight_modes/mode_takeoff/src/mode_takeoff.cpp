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
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_frame_transforms_lib/frame_transforms.h>

#include <mode_takeoff.hpp>

using namespace std::chrono_literals;
using namespace tucan_msgs::msg;
using namespace px4_msgs::msg;
using namespace px4_frame_transforms_lib::frame_transforms::utils::quaternion;

ModeTakeoff::ModeTakeoff() :
		Node("mode_takeoff"),
		mode_status_(MODE_INACTIVE),
		mode_status_publisher_{this->create_publisher<ModeStatus>("/mode_status", 1)},
		offboard_control_mode_publisher_{this->create_publisher<OffboardControlMode>("/offboard_control_mode", 1)},
		trajectory_setpoint_publisher_{this->create_publisher<TrajectorySetpoint>("/trajectory_setpoint", 1)}
{
	rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

	mission_state_subscriber = this->create_subscription<Mode>("/active_mode_id", 1, std::bind(&ModeTakeoff::mission_state_callback, this, std::placeholders::_1));
	
	
	vehicle_odom_subscriber_ = this->create_subscription<VehicleOdometry>("fmu/out/vehicle_odometry", qos, std::bind(&ModeTakeoff::vehicle_odom_callback, this, std::placeholders::_1));
	vehicle_status_subscriber_ = this->create_subscription<VehicleStatus>("fmu/out/vehicle_status", qos, std::bind(&ModeTakeoff::vehicle_status_callback, this, std::placeholders::_1));

	RCLCPP_INFO(this->get_logger(), "Starting Takeoff mode");

	timer_ = this->create_wall_timer(250ms, std::bind(&ModeTakeoff::timer_callback, this));
}

/**
 * @brief Timer callback to publish mode status. If mode is active, calls takeoff once and sets mode to finished.
 */
void ModeTakeoff::timer_callback()
{
	if (mode_status_ == MODE_ACTIVE)
	{
		publish_mode_status();

		publish_offboard_position_mode();

		if (!initiated_takeoff)
		{
			takeoff();
		}
		else
		{
			//print the current error
			RCLCPP_INFO(this->get_logger(), "Current error: %f", altitude_ + vehicle_odom_.position[2]);
			
			publish_trajectory_setpoint();
			if (!takeoff_finished && (altitude_ + vehicle_odom_.position[2]) < takeoff_tolerance)
			{
				takeoff_finished = true;
				RCLCPP_INFO(this->get_logger(), "Takeoff finished");
				mode_status_ = MODE_FINISHED;
				publish_mode_status();
				busy_ = false;
			}
		}
	}else if (mode_status_ == MODE_FINISHED)
	{
		publish_offboard_position_mode();
		publish_mode_status();
		publish_trajectory_setpoint();
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
	float z_position = takeoff_z_ - altitude_;
	TrajectorySetpoint msg{};
	msg.position = {takeoff_x_, takeoff_y_, z_position};
	msg.yaw = takeoff_yaw_; // [-PI:PI]
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
	msg.busy = busy_;
	mode_status_publisher_->publish(msg);
}

void ModeTakeoff::vehicle_odom_callback(const VehicleOdometry& msg)
{
	vehicle_odom_ = msg;
	vehicle_odom_received_ = true;
}

void ModeTakeoff::vehicle_status_callback(const VehicleStatus& msg)
{
	vehicle_status_ = msg;
	vehicle_status_received_ = true;
}

void ModeTakeoff::mission_state_callback(const Mode& msg)
{
	if (msg.mode_id == own_mode_id_ && mode_status_ == MODE_INACTIVE)
	{
		initiated_takeoff = false;
		busy_ = false;
		takeoff_finished = false;
		mode_status_ = MODE_ACTIVE;
		RCLCPP_INFO(this->get_logger(), "Takeoff mode started");
	}
}

void ModeTakeoff::takeoff(){
	if (vehicle_odom_received_ && vehicle_status_received_){
		if (vehicle_status_.nav_state == vehicle_status_.NAVIGATION_STATE_OFFBOARD){
			if (vehicle_status_.arming_state == vehicle_status_.ARMING_STATE_ARMED){
				if (!initiated_takeoff){
					takeoff_x_ = vehicle_odom_.position[0];
					takeoff_y_ = vehicle_odom_.position[1];
					takeoff_z_ = vehicle_odom_.position[2];
					initiated_takeoff = true;
					takeoff_finished = false;
					busy_ = true;
					takeoff_yaw_ = float(quaternion_get_yaw(array_to_eigen_quat(vehicle_odom_.q)));
				}
			}else{
				RCLCPP_INFO(this->get_logger(), "Vehicle not armed, waiting...");
			}
		}else{
			RCLCPP_INFO(this->get_logger(), "Vehicle not in offboard mode, waiting...");
		}
	}else{
		RCLCPP_INFO(this->get_logger(), "Local position not yet received from FCU, waiting...");
	}
	
}

int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ModeTakeoff>());

	rclcpp::shutdown();
	return 0;
}
