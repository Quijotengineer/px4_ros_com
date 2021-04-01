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

 * The TrajectorySetpoint message and the OFFBOARD mode in general are under an ongoing update.
 * Please refer to PR: https://github.com/PX4/PX4-Autopilot/pull/16739 for more info. 
 * As per PR: https://github.com/PX4/PX4-Autopilot/pull/17094, the format
 * of the TrajectorySetpoint message shall change.
 */

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardCommander : public rclcpp::Node {
public:
	OffboardCommander() : Node("offboard_commander_node") {
#ifdef ROS_DEFAULT_API
		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("OffboardControlMode_PubSubTopic", 10);
		trajectory_setpoint_publisher_ =
			this->create_publisher<TrajectorySetpoint>("TrajectorySetpoint_PubSubTopic", 10);
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("VehicleCommand_PubSubTopic", 10);
#else
		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("OffboardControlMode_PubSubTopic");
		trajectory_setpoint_publisher_ =
		 	this->create_publisher<TrajectorySetpoint>("TrajectorySetpoint_PubSubTopic");
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("VehicleCommand_PubSubTopic");
#endif

        target_trajectory_setpoint_subscriber_ = 
            this->create_subscription<geometry_msgs::msg::PoseStamped>("osd/next_trajectory_setpoint",
                 1, std::bind(&OffboardCommander::update_target_setpoint_cb, this, std::placeholders::_1));

		/* Obtain a syncronized timestamp to be set and sent with the offboard_control_mode
         * and trajectory_setpoint messages. */
		timesync_sub_ =
			this->create_subscription<px4_msgs::msg::Timesync>("Timesync_PubSubTopic", 10,
				[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
					timestamp_.store(msg->timestamp);
				});

		offboard_setpoint_counter_ = 0;

        // Define takeoff pose
        next_trajectory_setpoint_msg.timestamp = timestamp_.load();
        next_trajectory_setpoint_msg.x = 0.0;
        next_trajectory_setpoint_msg.y = 0.0;
        next_trajectory_setpoint_msg.z = -1.0;
        next_trajectory_setpoint_msg.yaw = -3.14; // [-PI:PI]
		std::cout << "Defined initial trajectory setpoint (x, y, z, yaw): " << next_trajectory_setpoint_msg.x << 
		next_trajectory_setpoint_msg.y << next_trajectory_setpoint_msg.z << next_trajectory_setpoint_msg.yaw << std::endl;

        /* The above is the main loop spining on the ROS 2 node. It first sends 10 setpoint
         * messages before sending the command to change to offboard mode At the same time,
         * both offboard_control_mode and trajectory_setpoint messages are sent to the flight controller. */
		auto timer_callback = [this]() -> void {
			
			// Change to Offboard mode after 10 setpoints
			if (offboard_setpoint_counter_ == 10) {
				
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();
			}

            // offboard_control_mode needs to be paired with trajectory_setpoint
			publish_offboard_control_mode();
			publish_trajectory_setpoint(); // should be doable with:
            // trajectory_setpoint_publisher_->publish(next_trajectory_setpoint_msg);

           	// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}


		};
		timer_ = this->create_wall_timer(33ms, timer_callback);
	}

	void arm() const;
	void disarm() const;

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_trajectory_setpoint_subscriber_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	void publish_offboard_control_mode() const;
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0) const;
    void update_target_setpoint_cb (const geometry_msgs::msg::PoseStamped::SharedPtr msg);
	void takeoff() const;
	void publish_trajectory_setpoint();

    px4_msgs::msg::TrajectorySetpoint next_trajectory_setpoint_msg;
};

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardCommander::publish_vehicle_command(uint16_t command, float param1,
					      float param2) const {
	VehicleCommand msg{};
	msg.timestamp = timestamp_.load();
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	vehicle_command_publisher_->publish(msg);
}


/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardCommander::arm() const {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}


/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardCommander::disarm() const {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}


/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */

void OffboardCommander::publish_offboard_control_mode() const {
	OffboardControlMode msg{};
	msg.timestamp = timestamp_.load();
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;

	offboard_control_mode_publisher_->publish(msg);
}


/**
 * The position is already being published in the NED coordinate frame for simplicity,
 * but in the case of the user wanting to subscribe to data coming from other nodes,
 * and since the standard frame of reference in ROS/ROS 2 is ENU, the user can use
 * the available helper functions in the frame_transform library.
*/
void OffboardCommander::update_target_setpoint_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {

	next_trajectory_setpoint_msg.timestamp = timestamp_.load();
    // Needs converting frames
	next_trajectory_setpoint_msg.x = msg->pose.position.x;
	next_trajectory_setpoint_msg.y = msg->pose.position.y;
	next_trajectory_setpoint_msg.z = -msg->pose.position.z; // ENU (ROS) to NED (PX4)
	next_trajectory_setpoint_msg.yaw = -3.14; // [-PI:PI]

	RCLCPP_INFO(this->get_logger(), "Updated next target trajectory setpoint");
}


void OffboardCommander::publish_trajectory_setpoint() {
	TrajectorySetpoint msg{};
	msg.timestamp = timestamp_.load();
	msg.x = next_trajectory_setpoint_msg.x;
	msg.y = next_trajectory_setpoint_msg.y;
	msg.z = next_trajectory_setpoint_msg.z;
	msg.yaw = next_trajectory_setpoint_msg.yaw; // [-PI:PI]

	trajectory_setpoint_publisher_->publish(msg);
}



int main(int argc, char* argv[]) {
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardCommander>());

	rclcpp::shutdown();
	return 0;
}