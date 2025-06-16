#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <memory>
#include <string>

class PoseTFBroadcaster : public rclcpp::Node
{
public:
  PoseTFBroadcaster()
  : Node("pose_tf_broadcaster")
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Gazebo pose 토픽들 구독
    sub_x1_static_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/model/X1_asp/pose_static", 10,
      std::bind(&PoseTFBroadcaster::pose_callback, this, std::placeholders::_1));

    sub_x1_dynamic_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/model/X1_asp/pose", 10,
      std::bind(&PoseTFBroadcaster::pose_callback, this, std::placeholders::_1));

    sub_x500_static_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/model/x500_gimbal_0/pose_static", 10,
      std::bind(&PoseTFBroadcaster::pose_callback, this, std::placeholders::_1));

    sub_x500_dynamic_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/model/x500_gimbal_0/pose", 10,
      std::bind(&PoseTFBroadcaster::pose_callback, this, std::placeholders::_1));
  }

private:
  void pose_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    for (auto transform : msg->transforms)
    {
      // "default" → "map" 프레임명 치환
      if (transform.header.frame_id == "default") {
        transform.header.frame_id = "map";
      }

      // 기본 TF 브로드캐스트
      tf_broadcaster_->sendTransform(transform);

      // X1_asp → odom 추가 브로드캐스트
      if (transform.child_frame_id == "X1_asp") {
        geometry_msgs::msg::TransformStamped odom_tf = transform;
        odom_tf.child_frame_id = "X1_asp/odom";
        tf_broadcaster_->sendTransform(odom_tf);
      }

      // base_link가 포함된 경우 → map → base_link 추가 브로드캐스트
      if (transform.child_frame_id.find("base_link") != std::string::npos) {
        geometry_msgs::msg::TransformStamped base_tf = transform;
        base_tf.header.frame_id = "map";
        base_tf.child_frame_id = "base_link";
        tf_broadcaster_->sendTransform(base_tf);
      }
    }
  }

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_x1_static_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_x1_dynamic_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_x500_static_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_x500_dynamic_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseTFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
