#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <tuple>
#include <cmath>
#include <algorithm>

using std::placeholders::_1;

class PID {
public:
  PID(double Kp, double Ki, double Kd)
  : Kp_(Kp), Ki_(Ki), Kd_(Kd), prev_error_(0.0), integral_(0.0) {}

  double compute(double error, double dt) {
    integral_ += error * dt;
    double derivative = dt > 0.0 ? (error - prev_error_) / dt : 0.0;
    double output = Kp_ * error + Ki_ * integral_ + Kd_ * derivative;
    prev_error_ = error;
    return output;
  }

private:
  double Kp_, Ki_, Kd_;
  double prev_error_, integral_;
};

class PathFollower : public rclcpp::Node {
public:
  PathFollower()
  : Node("path_follower"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_),
    pid_speed_(1.0, 0.0, 0.1), pid_steer_(2.0, 0.0, 0.1), current_target_index_(0)
  {
    declare_parameter<std::string>("path_file", "");
    declare_parameter<double>("arrival_threshold", 0.5);
    declare_parameter<double>("longitudinal_speed", 3.0);

    get_parameter("path_file", path_file_);
    get_parameter("arrival_threshold", arrival_thresh_);
    get_parameter("longitudinal_speed", max_speed_);

    RCLCPP_INFO(get_logger(), "[param check] path_file: %s", path_file_.c_str());
    RCLCPP_INFO(get_logger(), "[param check] arrival_thresh_: %.2f", arrival_thresh_);
    RCLCPP_INFO(get_logger(), "[param check] max_speed_: %.2f", max_speed_);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/model/X1_asp/cmd_vel", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("target_marker", 10);

    load_path();
    timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&PathFollower::control_loop, this));
  }

private:
  void load_path() {
    std::ifstream file(path_file_);
    if (!file.is_open()) {
      RCLCPP_ERROR(get_logger(), "❌ 경로 파일 열기 실패: %s", path_file_.c_str());
      return;
    }

    std::string line;
    std::getline(file, line); // header 제거
    while (std::getline(file, line)) {
      std::stringstream ss(line);
      std::string x_str, y_str, state_str;
      if (std::getline(ss, x_str, ',') && std::getline(ss, y_str, ',') && std::getline(ss, state_str)) {
        double x = std::stod(x_str);
        double y = std::stod(y_str);
        int state = std::stoi(state_str);
        path_.emplace_back(x, y, state);
      }
    }
    RCLCPP_INFO(get_logger(), "📍 %zu개의 waypoints 로드됨.", path_.size());
  }

  bool update_pose() {
    try {
      auto transform = tf_buffer_.lookupTransform("map", "X1_asp/base_link", tf2::TimePointZero);
      current_x_ = transform.transform.translation.x;
      current_y_ = transform.transform.translation.y;

      tf2::Quaternion q;
      tf2::fromMsg(transform.transform.rotation, q);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      current_yaw_ = yaw;
      return true;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(get_logger(), "TF 실패: %s", ex.what());
      return false;
    }
  }

  void control_loop() {
    if (!update_pose() || current_target_index_ >= path_.size()) return;

    auto [goal_x, goal_y, goal_state] = path_[current_target_index_];
    double dx = goal_x - current_x_;
    double dy = goal_y - current_y_;
    double dist = std::hypot(dx, dy);
    double target_yaw = std::atan2(dy, dx);
    double yaw_error = std::atan2(std::sin(target_yaw - current_yaw_), std::cos(target_yaw - current_yaw_));

    if (goal_state == 2 && dist < arrival_thresh_) {
      geometry_msgs::msg::Twist stop;
      cmd_pub_->publish(stop);
      RCLCPP_INFO(get_logger(), "🛑 state=2 지점 도착 → 정지");
      return;
    }

    double dt = 0.1;
    double speed = std::clamp(pid_speed_.compute(dist, dt), -max_speed_, max_speed_);
    double steer = std::clamp(pid_steer_.compute(yaw_error, dt), -1.0, 1.0);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = speed;
    cmd.angular.z = steer;
    cmd_pub_->publish(cmd);

    if (dist < arrival_thresh_ && current_target_index_ + 1 < path_.size()) {
      ++current_target_index_;
      RCLCPP_INFO(get_logger(), "➡️ 다음 waypoint로 이동 (%zu/%zu)", current_target_index_, path_.size());
    }

    publish_marker(goal_x, goal_y);
  }

  void publish_marker(double x, double y) {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = "map";
    marker.ns = "goal";
    marker.id = 0;
    marker.type = marker.SPHERE;
    marker.action = marker.ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.2;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.r = 1.0;
    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(0.3);
    marker_pub_->publish(marker);
  }

  std::string path_file_;
  double arrival_thresh_, max_speed_;
  double current_x_{0.0}, current_y_{0.0}, current_yaw_{0.0};
  size_t current_target_index_;
  std::vector<std::tuple<double, double, int>> path_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  PID pid_speed_, pid_steer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathFollower>());
  rclcpp::shutdown();
  return 0;
}
