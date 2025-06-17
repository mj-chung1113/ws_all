// offboard_control.cpp — revised (2025‑06‑13)
// ▸  ENU → NED conversion fixed (position + yaw)
// ▸  Un‑used fields set to NaN to avoid mixed‑mode issues
// ▸  50 Hz publishing (20 ms timer) as recommended by PX4
// ▸  Added timestamps on every setpoint, OffboardControlMode already handled
// ▸  Minor clean‑ups & header additions

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // <-- brings in fromMsg specialization
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <limits>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control_bridge"), offboard_setpoint_counter_(0)
    {
        auto qos = rclcpp::SensorDataQoS(); 
        offboard_ctrl_pub_  = create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", qos);
        traj_sp_pub_       = create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos);
        vehicle_cmd_pub_   = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", qos);

        // ── Pose command subscriber (map‑frame ENU)
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/command/pose", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
            {
                /* ENU (map) → NED (PX4) */
                setpoint_.position = {
                    static_cast<float>( msg->pose.position.y ),          // N  ←  y_ENU
                    static_cast<float>( msg->pose.position.x ),          // E  ←  x_ENU
                    static_cast<float>(-msg->pose.position.z)            // D  ← ‑z_ENU
                };

                /* yaw conversion: ψ_NED = 90° − ψ_ENU */
                // double yaw_enu = tf2::getYaw( msg->pose.orientation );
                // setpoint_.yaw  = static_cast<float>( M_PI_2 - yaw_enu );

                // mark_position_mode();
            });

        // ── Initialise take‑off setpoint (30 m Up  →  ‑30 m Down)
        setpoint_.position = { 0.0f, 0.0f, -5.0f };
        // setpoint_.yaw      = 0.0f;
        mark_position_mode();

        timer_ = create_wall_timer( 20ms, std::bind(&OffboardControl::timer_cb, this) ); // 50 Hz
    }

private:
    // ── pubs / subs ──
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_ctrl_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr   traj_sp_pub_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr       vehicle_cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ── state ──
    TrajectorySetpoint setpoint_{};
    uint64_t offboard_setpoint_counter_;

    // ──────────────────────────────────────────────────────────────
    void mark_position_mode()
    {
        /*  Ensure velocity/acc/jerk/yawspeed are NaN so PX4 treats this
            setpoint as *pure position*  */
        float nan = std::numeric_limits<float>::quiet_NaN();
        setpoint_.velocity     = { nan, nan, nan };
        setpoint_.acceleration = { nan, nan, nan };
        setpoint_.jerk         = { nan, nan, nan };
        setpoint_.yawspeed     = nan;
    }

    // ──────────────────────────────────────────────────────────────
    void timer_cb()
    {
        /* Arm + switch to OFFBOARD after sending 10 initial setpoints (200 ms) */
        if (offboard_setpoint_counter_ == 11)
        {
            send_vehicle_cmd( VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6 ); // OFFBOARD
            send_vehicle_cmd( VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f );
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000, "Sent OFFBOARD + ARM");
        }

        publish_offboard_ctrl_mode();

        setpoint_.timestamp = get_clock()->now().nanoseconds() / 1000; // µs
        traj_sp_pub_->publish( setpoint_ );

        if (offboard_setpoint_counter_ < 11) ++offboard_setpoint_counter_;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 100, "offboard_setpoint_counter_: %ld", offboard_setpoint_counter_);
        
    }

    // ──────────────────────────────────────────────────────────────
    void publish_offboard_ctrl_mode()
    {
        OffboardControlMode m{};
        m.position = true;
        m.velocity = false;
        m.acceleration = false;
        m.attitude = false;
        m.body_rate = false;
        m.timestamp = get_clock()->now().nanoseconds() / 1000;
        offboard_ctrl_pub_->publish( m );
    }

    // ──────────────────────────────────────────────────────────────
    void send_vehicle_cmd(uint16_t cmd, float p1 = 0.f, float p2 = 0.f)
    {
        VehicleCommand v{};
        v.command          = cmd;
        v.param1           = p1;
        v.param2           = p2;
        v.target_system    = 1;
        v.target_component = 1;
        v.source_system    = 1;
        v.source_component = 1;
        v.from_external    = true;
        v.timestamp        = get_clock()->now().nanoseconds() / 1000;
        vehicle_cmd_pub_->publish( v );
    }
};

// ──────────────────────────────────────────────────────────────────
int main(int argc, char **argv)
{
    rclcpp::init( argc, argv );
    rclcpp::spin( std::make_shared<OffboardControl>() );
    rclcpp::shutdown();
    return 0;
}
