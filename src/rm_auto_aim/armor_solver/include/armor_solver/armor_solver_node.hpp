// Copyright (C) FYT Vision Group. Licensed under Apache License 2.0.
//
// Additional modifications and features by Lori Lai. Licensed under Apache License 2.0.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ARMOR_SOLVER__ARMOR_SOLVER_NODE_HPP_
#define ARMOR_SOLVER__ARMOR_SOLVER_NODE_HPP_

// ros2
#include <auto_aim_interfaces/msg/detail/target__struct.hpp>
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
// std
#include <memory>
#include <string>
#include <vector>
// project
#include "armor_solver/armor_solver.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/measurement.hpp"
#include "auto_aim_interfaces/msg/target.hpp"

namespace rm_auto_aim {
using tf2_filter = tf2_ros::MessageFilter<auto_aim_interfaces::msg::Target>;
class ArmorSolverNode : public rclcpp::Node {
public:
  explicit ArmorSolverNode(const rclcpp::NodeOptions &options);

private:
  void targetCallback(const auto_aim_interfaces::msg::Target::SharedPtr target_ptr);

  void initMarkers() noexcept;

  void publishMarkers(const auto_aim_interfaces::msg::GimbalCmd &gimbal_cmd) noexcept;
  
  bool debug_mode_;

  // The time when the last message was received
  rclcpp::Time last_time_;
  double dt_;

  // Armor Solver
  std::unique_ptr<Solver> solver_;

  // Subscriber with tf2 message_filter
  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<auto_aim_interfaces::msg::Target> target_sub_;
  auto_aim_interfaces::msg::Target armor_target_;
  std::shared_ptr<tf2_filter> tf2_filter_;

  // Measurement publisher
  rclcpp::Publisher<auto_aim_interfaces::msg::Measurement>::SharedPtr measure_pub_;

  // Publisher
  rclcpp::Publisher<auto_aim_interfaces::msg::GimbalCmd>::SharedPtr gimbal_pub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  void timerCallback();
  
  // Enable/Disable Armor Solver
  bool enable_;
//   rclcpp::Service<auto_aim_interfaces::srv::SetMode>::SharedPtr set_mode_srv_;

  // Visualization marker publisher
  visualization_msgs::msg::Marker position_marker_;
  visualization_msgs::msg::Marker linear_v_marker_;
  visualization_msgs::msg::Marker angular_v_marker_;
  visualization_msgs::msg::Marker trajectory_marker_;
  visualization_msgs::msg::Marker armors_marker_;
  visualization_msgs::msg::Marker selection_marker_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_SOLVER_SOLVER_NODE_HPP_
