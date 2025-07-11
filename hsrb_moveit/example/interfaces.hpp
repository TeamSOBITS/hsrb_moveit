#ifndef HSRB_MOVEIT_EXAMPLE_INTERFACES_HPP_
#define HSRB_MOVEIT_EXAMPLE_INTERFACES_HPP_

#include <moveit/move_group_interface/move_group_interface.h>

#include <rclcpp/rclcpp.hpp>

namespace hsrb_moveit {

struct Interfaces {
  moveit::planning_interface::MoveGroupInterface arm_group;
  moveit::planning_interface::MoveGroupInterface head_group;
  moveit::planning_interface::MoveGroupInterface base_group;

  explicit Interfaces(const rclcpp::Node::SharedPtr& node, double scaling_factor = 0.5)
      : arm_group(node, "arm"),
        head_group(node, "head"),
        base_group(node, "base") {
    arm_group.setMaxVelocityScalingFactor(scaling_factor);
    arm_group.setMaxAccelerationScalingFactor(scaling_factor);
    head_group.setMaxVelocityScalingFactor(scaling_factor);
    head_group.setMaxAccelerationScalingFactor(scaling_factor);
    base_group.setMaxVelocityScalingFactor(scaling_factor);
    base_group.setMaxAccelerationScalingFactor(scaling_factor);
  }

  bool MoveToNeutral() {
    if (!arm_group.setNamedTarget("neutral")) {
      return false;
    }
    arm_group.move();

    if (!head_group.setNamedTarget("neutral")) {
      return false;
    }
    head_group.move();

    if (!base_group.setNamedTarget("neutral")) {
      return false;
    }
    base_group.move();

    return true;
  }
};

}
#endif