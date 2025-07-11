#include "interfaces.hpp"
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "sobits_interfaces/srv/get_hand_to_target_coord.hpp"

class MoveItIkDemoService : public rclcpp::Node {
public:
  MoveItIkDemoService()
  : Node("moveit_to_coord_service")
  {
    service_ = this->create_service<sobits_interfaces::srv::GetHandToTargetCoord>(
      "/moveit_to_coord",
      std::bind(&MoveItIkDemoService::handle_service_request, this,
                std::placeholders::_1, std::placeholders::_2));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/omni_base_controller/wheel_odom", 10,
      std::bind(&MoveItIkDemoService::odom_callback, this, std::placeholders::_1));
  }

  void initialize()
  {
    whole_body_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "whole_body");
    whole_body_group_->setMaxVelocityScalingFactor(0.5);
    whole_body_group_->setMaxAccelerationScalingFactor(0.5);

    interfaces_ = std::make_unique<hsrb_moveit::Interfaces>(shared_from_this());

    if (!interfaces_->MoveToNeutral()) {
      RCLCPP_ERROR(this->get_logger(), "MoveToNeutral failure on startup");
    }
  }

private:
  rclcpp::Service<sobits_interfaces::srv::GetHandToTargetCoord>::SharedPtr service_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> whole_body_group_;
  std::unique_ptr<hsrb_moveit::Interfaces> interfaces_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  double base_x_ = 0.0;
  double base_y_ = 0.0;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    base_x_ = msg->pose.pose.position.x;
    base_y_ = msg->pose.pose.position.y;
  }

  void handle_service_request(
    const std::shared_ptr<sobits_interfaces::srv::GetHandToTargetCoord::Request> request,
    std::shared_ptr<sobits_interfaces::srv::GetHandToTargetCoord::Response> response) {

    double goal_x = request->target_coord.transform.translation.x;
    double goal_y = request->target_coord.transform.translation.y;
    double goal_z = request->target_coord.transform.translation.z;

    double dx = goal_x - base_x_;
    double dy = goal_y - base_y_;
    double yaw = std::atan2(dy, dx) + M_PI;

    double pitch_angle = -M_PI / 2.0;
    double roll_angle = 0.0;

    tf2::Quaternion q;
    q.setRPY(roll_angle, pitch_angle, yaw);
    geometry_msgs::msg::Quaternion orientation = tf2::toMsg(q);

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = goal_x;
    target_pose.position.y = goal_y;
    target_pose.position.z = goal_z;
    target_pose.orientation = orientation;

    whole_body_group_->clearPathConstraints();

    moveit_msgs::msg::JointConstraint jc;
    jc.joint_name = "arm_roll_joint";
    jc.position = 0.0;
    jc.tolerance_above = 0.1;
    jc.tolerance_below = 0.1;
    jc.weight = 1.0;

    moveit_msgs::msg::Constraints path_constraints;
    path_constraints.joint_constraints.push_back(jc);
    whole_body_group_->setPathConstraints(path_constraints);

    whole_body_group_->setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (whole_body_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      whole_body_group_->execute(my_plan);
      response->success = true;
      response->message = "Motion plan and execution successful.";
    } else {
      response->success = false;
      response->message = "Failed to plan motion to target coordinate.";
    }

    response->move_pose = target_pose;
    response->target_joint_names.clear();
    response->target_joint_rad.clear();

    whole_body_group_->clearPathConstraints();
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto move_it_ik_demo_service_node = std::make_shared<MoveItIkDemoService>();
  move_it_ik_demo_service_node->initialize();

  rclcpp::spin(move_it_ik_demo_service_node);
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}