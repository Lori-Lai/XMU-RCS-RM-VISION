// Copyright (C) FYT Vision Group. Licensed under Apache License 2.0.

#include "armor_solver/trajectory_compensator.hpp"

namespace rm_auto_aim {
bool TrajectoryCompensator::compensate(const Eigen::Vector3d &target_position,
                                       double &pitch) const noexcept {
  double target_height = target_position(2);
  // The iterative_height is used to calculate angle in each iteration
  double iterative_height = target_height;
  double impact_height = 0;
  double distance =
    std::sqrt(target_position(0) * target_position(0) + target_position(1) * target_position(1));
  double angle = std::atan2(target_height, distance);
  double dh = 0;
  // Iterate to find the right angle, which makes the impact height equal to the
  // target height
  for (int i = 0; i < iteration_times; ++i) {
    angle = std::atan2(iterative_height, distance);
    if (std::abs(angle) > M_PI / 2.5) {
      break;
    }
    impact_height = calculateTrajectory(distance, angle);
    dh = target_height - impact_height;
    if (std::abs(dh) < 0.01) {
      break;
    }
    iterative_height += dh;
  }
  if (std::abs(dh) > 0.01 || std::abs(angle) > M_PI / 2.5) {
    return false;
  }
  pitch = angle;
  return true;
}

std::vector<std::pair<double, double>> TrajectoryCompensator::getTrajectory(
  double distance, double angle) const noexcept {
  std::vector<std::pair<double, double>> trajectory;

  if (distance < 0) {
    return trajectory;
  }

  for (double x = 0; x < distance; x += 0.03) {
    trajectory.emplace_back(x, calculateTrajectory(x, angle));
  }
  return trajectory;
}

double IdealCompensator::calculateTrajectory(const double x, const double angle) const noexcept {
  double t = x / (velocity * cos(angle));
  double y = velocity * sin(angle) * t - 0.5 * gravity * t * t;
  return y;
}

double IdealCompensator::getFlyingTime(const Eigen::Vector3d &target_position) const noexcept {
  double distance =
    sqrt(target_position(0) * target_position(0) + target_position(1) * target_position(1));
  double angle = atan2(target_position(2), distance);
  double t = distance / (velocity * cos(angle));
  return t;
}

double ResistanceCompensator::calculateTrajectory(const double x,
                                                  const double angle) const noexcept {
  double r = resistance < 1e-4 ? 1e-4 : resistance;
  double t = (exp(r * x) - 1) / (r * velocity * cos(angle));
  double y = velocity * sin(angle) * t - 0.5 * gravity * t * t;
  return y;
}

double ResistanceCompensator::getFlyingTime(const Eigen::Vector3d &target_position) const noexcept {
  double r = resistance < 1e-4 ? 1e-4 : resistance;
  double distance =
    sqrt(target_position(0) * target_position(0) + target_position(1) * target_position(1));
  double angle = atan2(target_position(2), distance);
  double t = (exp(r * distance) - 1) / (r * velocity * cos(angle));
  return t;
}
}  // namespace rm_auto_aim
