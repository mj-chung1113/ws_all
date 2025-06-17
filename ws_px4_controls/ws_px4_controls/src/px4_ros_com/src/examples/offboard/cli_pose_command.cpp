#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <cstdlib>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("cli_pose_command");
  auto pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/command/pose", 10);

  if (argc != 5) {
    RCLCPP_INFO(node->get_logger(), "Usage: cli_pose_command x y z yaw_deg");
    return 1;
  }

  double x = std::stod(argv[1]);
  double y = std::stod(argv[2]);
  double z = std::stod(argv[3]);
  double yaw_deg = std::stod(argv[4]);
  double yaw_rad = yaw_deg * M_PI / 180.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw_rad);
  q.normalize();

  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = node->now();
  msg.header.frame_id = "map";
  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = z;
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();
  msg.pose.orientation.w = q.w();

  rclcpp::Rate rate(10);
  for (int i = 0; i < 10; ++i) {
    pub->publish(msg);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
