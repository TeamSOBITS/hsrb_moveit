#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

#include <geometry_msgs/msg/pose.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "sobits_interfaces/srv/get_hand_to_target_tf.hpp"
#include "interfaces.hpp"

using GetHandToTargetTF = sobits_interfaces::srv::GetHandToTargetTF;

class MoveitIKServiceNode : public rclcpp::Node
{
public:
  MoveitIKServiceNode()
  : Node("moveit_ik_service_node")
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    service_ = this->create_service<GetHandToTargetTF>(
        "moveit_to_tf",
        std::bind(&MoveitIKServiceNode::callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "MoveIt IK Service Ready");
  }

private:
  void callback(
      const std::shared_ptr<GetHandToTargetTF::Request> request,
      std::shared_ptr<GetHandToTargetTF::Response> response)
  {
    auto logger = this->get_logger();

    if (!initialized_) {
      interfaces_ = std::make_shared<hsrb_moveit_config::Interfaces>(shared_from_this());

      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          shared_from_this(), "whole_body_weighted");

      move_group_->setMaxVelocityScalingFactor(0.5);
      move_group_->setMaxAccelerationScalingFactor(0.5);

      initialized_ = true;
    }

    RCLCPP_INFO(logger, "Target TF: %s", request->target_frame.c_str());

    if (!interfaces_->MoveToNeutral()) {
      response->success = false;
      response->message = "MoveToNeutral failed";
      return;
    }

    rclcpp::sleep_for(std::chrono::seconds(1));

    geometry_msgs::msg::Pose target_pose;

    try
    {
      auto transform = tf_buffer_->lookupTransform(
          "odom",
          request->target_frame,
          tf2::TimePointZero);

      target_pose.position.x = transform.transform.translation.x;
      target_pose.position.y = transform.transform.translation.y;
      target_pose.position.z = transform.transform.translation.z;
      target_pose.orientation = transform.transform.rotation;
    }
    catch (tf2::TransformException &ex)
    {
      response->success = false;
      response->message = ex.what();
      return;
    }

    move_group_->setPoseTarget(target_pose);

    bool ok = (move_group_->move() ==
               moveit::core::MoveItErrorCode::SUCCESS);

    // debug: MoveToNeutral
    // if (!interfaces_->MoveToNeutral()) {
    //   response->success = false;
    //   response->message = "MoveToNeutral failed";
    //   return;
    // }

    if (!ok) {
      response->success = false;
      response->message = "MoveIt failed";
      return;
    }

    response->success = true;
    response->message = "Success";
  }

  rclcpp::Service<GetHandToTargetTF>::SharedPtr service_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<hsrb_moveit_config::Interfaces> interfaces_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool initialized_ = false;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MoveitIKServiceNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}