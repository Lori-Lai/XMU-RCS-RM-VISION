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

#include "armor_solver/armor_solver_node.hpp"

// std
#include <memory>
#include <rclcpp/logging.hpp>
#include <vector>

namespace rm_auto_aim {
ArmorSolverNode::ArmorSolverNode(const rclcpp::NodeOptions &options)
: Node("armor_solver", options), solver_(nullptr) {
  RCLCPP_INFO(this->get_logger(), "ArmorSolverNode started!");

  debug_mode_ = this->declare_parameter("debug", true);

  // Subscriber with tf2 message_filter
  // tf2 relevant
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // Create the timer interface before call to waitForTransform,
  // to avoid a tf2_ros::CreateTimerInterfaceException exception
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  // subscriber and filter
  target_sub_.subscribe(this, "tracker/target", rmw_qos_profile_sensor_data);
  target_frame_ = this->declare_parameter("target_frame", "odom");
  tf2_filter_ = std::make_shared<tf2_filter>(target_sub_,
                                             *tf2_buffer_,
                                             target_frame_,
                                             10,
                                             this->get_node_logging_interface(),
                                             this->get_node_clock_interface(),
                                             std::chrono::duration<int>(1));
  // Register a callback with tf2_ros::MessageFilter to be called when
  // transforms are available
  tf2_filter_->registerCallback(&ArmorSolverNode::targetCallback, this);

  // Measurement publisher (for debug usage)
  measure_pub_ = this->create_publisher<auto_aim_interfaces::msg::Measurement>("solver/measurement",
                                                                         rclcpp::SensorDataQoS());

  gimbal_pub_ = this->create_publisher<auto_aim_interfaces::msg::GimbalCmd>("solver/cmd_gimbal",
                                                                      rclcpp::SensorDataQoS());
  // Timer 250 Hz
  pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(4),
                                       std::bind(&ArmorSolverNode::timerCallback, this));
  armor_target_.header.frame_id = "";

  // Enable/Disable Armor Solver
  enable_ = true;

  if (debug_mode_) {
    initMarkers();
  }
}

void ArmorSolverNode::timerCallback() {
  // RCLCPP_INFO(this->get_logger(), "ArmorSolverNode timerCallback");
  if (solver_ == nullptr) {
    return;
  }

  if (!enable_) {
    return;
  }

  // Init message
  auto_aim_interfaces::msg::GimbalCmd control_msg;

  // If target never detected
  if (armor_target_.header.frame_id.empty()) {
    control_msg.yaw_diff = 0;
    control_msg.pitch_diff = 0;
    control_msg.distance = -1;
    control_msg.pitch = 0;
    control_msg.yaw = 0;
    control_msg.fire_advice = false;
    gimbal_pub_->publish(control_msg);
    return;
  }

  if (armor_target_.tracking) {
    try {
      // RCLCPP_INFO(this->get_logger(), "ArmorSolverNode try to solve");
      control_msg = solver_->solve(armor_target_, this->now(), tf2_buffer_);
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Something went wrong in solver!");
      control_msg.yaw_diff = 0;
      control_msg.pitch_diff = 0;
      control_msg.distance = -1;
      control_msg.fire_advice = false;
    }
  } else {
    control_msg.yaw_diff = 0;
    control_msg.pitch_diff = 0;
    control_msg.distance = -1;
    control_msg.fire_advice = false;
  }
  gimbal_pub_->publish(control_msg);

  if (debug_mode_) {
    publishMarkers(control_msg);
  }
}

void ArmorSolverNode::initMarkers() noexcept {
  // Visualization Marker Publisher
  // See http://wiki.ros.org/rviz/DisplayTypes/Marker
  position_marker_.ns = "position";
  position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
  position_marker_.color.a = 1.0;
  position_marker_.color.g = 1.0;
  linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  linear_v_marker_.ns = "linear_v";
  linear_v_marker_.scale.x = 0.03;
  linear_v_marker_.scale.y = 0.05;
  linear_v_marker_.color.a = 1.0;
  linear_v_marker_.color.r = 1.0;
  linear_v_marker_.color.g = 1.0;
  angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  angular_v_marker_.ns = "angular_v";
  angular_v_marker_.scale.x = 0.03;
  angular_v_marker_.scale.y = 0.05;
  angular_v_marker_.color.a = 1.0;
  angular_v_marker_.color.b = 1.0;
  angular_v_marker_.color.g = 1.0;
  armors_marker_.ns = "filtered_armors";
  armors_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armors_marker_.scale.x = 0.03;
  armors_marker_.scale.z = 0.125;
  armors_marker_.color.a = 1.0;
  armors_marker_.color.b = 1.0;
  selection_marker_.ns = "selection";
  selection_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  selection_marker_.scale.x = selection_marker_.scale.y = selection_marker_.scale.z = 0.1;
  selection_marker_.color.a = 1.0;
  selection_marker_.color.g = 1.0;
  selection_marker_.color.r = 1.0;
  trajectory_marker_.ns = "trajectory";
  trajectory_marker_.type = visualization_msgs::msg::Marker::POINTS;
  trajectory_marker_.scale.x = 0.01;
  trajectory_marker_.scale.y = 0.01;
  trajectory_marker_.color.a = 1.0;
  trajectory_marker_.color.r = 1.0;
  trajectory_marker_.color.g = 0.75;
  trajectory_marker_.color.b = 0.79;
  trajectory_marker_.points.clear();

  marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("armor_solver/marker", 10);
}

void ArmorSolverNode::targetCallback(const auto_aim_interfaces::msg::Target::SharedPtr target_ptr) {
  // Lazy initialize solver owing to weak_from_this() can't be called in constructor
  if (solver_ == nullptr) {
    solver_ = std::make_unique<Solver>(weak_from_this());
  }

  // Init message
  auto_aim_interfaces::msg::Measurement measure_msg;
  rclcpp::Time time = target_ptr->header.stamp;

  armor_target_ = *target_ptr;

  last_time_ = time;
}

void ArmorSolverNode::publishMarkers(const auto_aim_interfaces::msg::GimbalCmd &gimbal_cmd) noexcept {
  selection_marker_.header = gimbal_cmd.header;
  trajectory_marker_.header = gimbal_cmd.header;

  visualization_msgs::msg::MarkerArray marker_array;

  if (armor_target_.tracking) {
    selection_marker_.action = visualization_msgs::msg::Marker::ADD;
    selection_marker_.points.clear();
    selection_marker_.pose.position.y = gimbal_cmd.distance * sin(gimbal_cmd.yaw * M_PI / 180);
    selection_marker_.pose.position.x = gimbal_cmd.distance * cos(gimbal_cmd.yaw * M_PI / 180);
    selection_marker_.pose.position.z = gimbal_cmd.distance * sin(gimbal_cmd.pitch * M_PI / 180);

    trajectory_marker_.action = visualization_msgs::msg::Marker::ADD;
    trajectory_marker_.points.clear();
    trajectory_marker_.header.frame_id = "gimbal_link";
    for (const auto &point : solver_->getTrajectory()) {
      geometry_msgs::msg::Point p;
      p.x = point.first;
      p.z = point.second;
      trajectory_marker_.points.emplace_back(p);
    }
    if (gimbal_cmd.fire_advice) {
      trajectory_marker_.color.r = 0;
      trajectory_marker_.color.g = 1;
      trajectory_marker_.color.b = 0;
    } else {
      trajectory_marker_.color.r = 1;
      trajectory_marker_.color.g = 1;
      trajectory_marker_.color.b = 1;
    }

  } else {
    trajectory_marker_.action = visualization_msgs::msg::Marker::DELETE;
    selection_marker_.action = visualization_msgs::msg::Marker::DELETE;
  }

  marker_array.markers.emplace_back(trajectory_marker_);
  marker_array.markers.emplace_back(selection_marker_);
  marker_pub_->publish(marker_array);
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorSolverNode)
