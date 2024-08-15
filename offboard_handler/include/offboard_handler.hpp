#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>

#include <rclcpp/rclcpp.hpp>

using namespace px4_msgs::msg;

class OffboardHandler : public rclcpp::Node
{
    public:
        OffboardHandler();
    
    private:
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
        rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;

        rclcpp::Subscription<TrajectorySetpoint>::SharedPtr command_subscriber_;
        rclcpp::Subscription<OffboardControlMode>::SharedPtr control_mode_subscriber_;

        // Reference trackers
        TrajectorySetpoint setpoint_;
        OffboardControlMode control_mode_;

        std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

        uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

        void loop_once();
        void command_callback(const TrajectorySetpoint::SharedPtr msg);
        void control_mode_callback(const OffboardControlMode::SharedPtr msg);
        void publish_offboard_control_mode();
        void publish_trajectory_setpoint();

};