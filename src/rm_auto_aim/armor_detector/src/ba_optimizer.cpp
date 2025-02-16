// Copyright Chen Jun 2023. Licensed under the MIT License.
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

#include "armor_detector/ba_optimizer.hpp"
// std
#include <memory>
// g2o
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/slam3d/types_slam3d.h>
// 3rd party
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
// project
#include "armor_detector/armor.hpp"

namespace rm_auto_aim {
G2O_USE_OPTIMIZATION_LIBRARY(dense)

BaOptimizer::BaOptimizer(const std::array<double, 9> &camera_matrix,
                   const std::vector<double> &dist_coeffs) {
  K_ = Eigen::Matrix3d::Identity();
  K_(0, 0) = camera_matrix[0];
  K_(1, 1) = camera_matrix[4];
  K_(0, 2) = camera_matrix[2];
  K_(1, 2) = camera_matrix[5];

  // Optimization information
  optimizer_.setVerbose(false);
  // Optimization method
  optimizer_.setAlgorithm(
      g2o::OptimizationAlgorithmFactory::instance()->construct(
          "lm_dense", solver_property_));
  // Initial step size
  lm_algorithm_ = dynamic_cast<g2o::OptimizationAlgorithmLevenberg *>(
      const_cast<g2o::OptimizationAlgorithm *>(optimizer_.algorithm()));
  lm_algorithm_->setUserLambdaInit(0.1);

  // Unit: m
  constexpr double small_half_y = SMALL_ARMOR_WIDTH / 2.0 / 1000.0;
  constexpr double small_half_z = SMALL_ARMOR_HEIGHT / 2.0 / 1000.0;
  constexpr double large_half_y = LARGE_ARMOR_WIDTH / 2.0 / 1000.0;
  constexpr double large_half_z = LARGE_ARMOR_HEIGHT / 2.0 / 1000.0;

  // Start from bottom left in clockwise order
  // Model coordinate: x forward, y left, z up
  small_armor_points_.emplace_back(cv::Point3f(0, small_half_y, -small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, small_half_y, small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, -small_half_z));

  large_armor_points_.emplace_back(cv::Point3f(0, large_half_y, -large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, large_half_y, large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, -large_half_y, large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, -large_half_y, -large_half_z));
}

Eigen::Matrix3d BaOptimizer::solveBa(const Armor &armor, const Eigen::Vector3d &t_camera_armor,
                  const Eigen::Matrix3d &R_camera_armor,
                  const Eigen::Matrix3d &R_imu_camera){
  // Reset optimizer
  optimizer_.clear();

  // Essential coordinate system transformation
  Eigen::Matrix3d R_imu_armor = R_imu_camera * R_camera_armor;
  Sophus::SO3d R_camera_imu = Sophus::SO3d(R_imu_camera.transpose());

  // Compute the initial yaw from rotation matrix
  double initial_armor_yaw;
  auto theta_by_sin = std::asin(-R_imu_armor(0, 1));
  auto theta_by_cos = std::acos(R_imu_armor(1, 1));
  if (std::abs(theta_by_sin) > 1e-5) {
    initial_armor_yaw = theta_by_sin > 0 ? theta_by_cos : -theta_by_cos;
  } else {
    initial_armor_yaw = R_imu_armor(1, 1) > 0 ? 0 : CV_PI;
  }

  // Get the pitch angle of the armor
  double armor_pitch =
      armor.number == "outpost" ? -FIFTTEN_DEGREE_RAD : FIFTTEN_DEGREE_RAD;
  Sophus::SO3d R_pitch = Sophus::SO3d::exp(Eigen::Vector3d(0, armor_pitch, 0));

  // Get the 3D points of the armor
  const auto object_points = armor.type == ArmorType::SMALL ? small_armor_points_ : large_armor_points_;

  // Fill the optimizer
  size_t id_counter = 0;

  VertexYaw *v_yaw = new VertexYaw();
  v_yaw->setId(id_counter++);
  v_yaw->setEstimate(initial_armor_yaw);
  optimizer_.addVertex(v_yaw);

  std::vector<cv::Point2f> image_armor_points;
  // Fill in image points
  image_armor_points.emplace_back(armor.left_light.bottom);
  image_armor_points.emplace_back(armor.left_light.top);
  image_armor_points.emplace_back(armor.right_light.top);
  image_armor_points.emplace_back(armor.right_light.bottom);

  for (size_t i = 0; i < 4; i++) {
    g2o::VertexPointXYZ *v_point = new g2o::VertexPointXYZ();
    v_point->setId(id_counter++);
    v_point->setEstimate(Eigen::Vector3d(
        object_points[i].x, object_points[i].y, object_points[i].z));
    v_point->setFixed(true);
    optimizer_.addVertex(v_point);

    EdgeProjection *edge =
        new EdgeProjection(R_camera_imu, R_pitch, t_camera_armor, K_);
    edge->setId(id_counter++);
    edge->setVertex(0, v_yaw);
    edge->setVertex(1, v_point);
    edge->setMeasurement(Eigen::Vector2d(image_armor_points[i].x, image_armor_points[i].y));
    edge->setInformation(EdgeProjection::InfoMatrixType::Identity());
    edge->setRobustKernel(new g2o::RobustKernelHuber);
    optimizer_.addEdge(edge);
  }

  // Start optimizing
  optimizer_.initializeOptimization();
  optimizer_.optimize(20);

  // Get yaw angle after optimization
  double yaw_optimized = v_yaw->estimate();

  if (std::isnan(yaw_optimized)) {
    RCLCPP_ERROR(rclcpp::get_logger("ba_optimizer"), "yaw_optimized is nan");
    return R_camera_armor;
  }

  // RCLCPP_INFO(rclcpp::get_logger("ba_optimizer"), "yaw_optimized: %f", yaw_optimized);
  
  Sophus::SO3d R_yaw = Sophus::SO3d::exp(Eigen::Vector3d(0, 0, yaw_optimized));
  return (R_camera_imu * R_yaw * R_pitch).matrix();
}

void VertexYaw::oplusImpl(const double *update) {
  Sophus::SO3d R_yaw = Sophus::SO3d::exp(Eigen::Vector3d(0, 0, update[0])) *
                       Sophus::SO3d::exp(Eigen::Vector3d(0, 0, _estimate));
  _estimate = R_yaw.log()(2);
}

EdgeProjection::EdgeProjection(const Sophus::SO3d &R_camera_imu,
                               const Sophus::SO3d &R_pitch,
                               const Eigen::Vector3d &t,
                               const Eigen::Matrix3d &K)
    : R_camera_imu_(R_camera_imu), R_pitch_(R_pitch), t_(t), K_(K) {}

void EdgeProjection::computeError() {
  // Get the rotation
  double yaw = static_cast<VertexYaw *>(_vertices[0])->estimate();
  Sophus::SO3d R_yaw = Sophus::SO3d::exp(Eigen::Vector3d(0, 0, yaw));
  Sophus::SO3d R = R_camera_imu_ * R_yaw * R_pitch_;

  // Get the 3D point
  Eigen::Vector3d p_3d =
      static_cast<g2o::VertexPointXYZ *>(_vertices[1])->estimate();

  // Get the observed 2D point
  Eigen::Vector2d obs(_measurement);

  // Project the 3D point to the 2D point
  Eigen::Vector3d p_2d = R * p_3d + t_;
  p_2d = K_ * (p_2d / p_2d.z());

  // Calculate the error
  _error = obs - p_2d.head<2>();
}

} // namespace fyt::auto_aim