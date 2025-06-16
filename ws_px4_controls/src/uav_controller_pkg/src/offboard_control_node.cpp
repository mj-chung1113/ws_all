// offboard_control.cpp (최종 버전)

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
    public:
    OffboardControl() : Node("offboard_control_bridge")
    {
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/command/pose", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                // 올바른 ENU -> NED 변환 규칙
                // ENU의 y (North) -> NED의 x (North)
                // ENU의 x (East)  -> NED의 y (East)
                // ENU의 z (Up)    -> NED의 z (Down)은 부호 반전
                setpoint_.position = {
                    (float)msg->pose.position.y,
                    (float)msg->pose.position.x,
                    -(float)msg->pose.position.z
                };
                setpoint_.yaw = -3.14; 
            });

        setpoint_.position = {0.0, 0.0, -30.0}; // 초기 이륙 고도 30m
        setpoint_.yaw = -3.14;
        offboard_setpoint_counter_ = 0;
        timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::timer_callback, this));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;

    uint64_t offboard_setpoint_counter_;
    TrajectorySetpoint setpoint_{};

    void timer_callback()
    {
        if (offboard_setpoint_counter_ == 10) {
            this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
        }

        publish_offboard_control_mode();
        
        setpoint_.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_publisher_->publish(setpoint_);

        if (offboard_setpoint_counter_ < 11) {
            offboard_setpoint_counter_++;
        }
    }

    void publish_offboard_control_mode()
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

    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0)
    {
        VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_publisher_->publish(msg);
    }
};

int main(int argc, char *argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}
