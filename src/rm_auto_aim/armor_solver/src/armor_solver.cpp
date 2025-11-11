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

#include "armor_solver/armor_solver.hpp"
// std
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>
#include <stdexcept>
// ros2
#include <rclcpp/logging.hpp>
// Eigen
#include <Eigen/Geometry>
// tf2
#include <tf2_eigen/tf2_eigen.hpp>
// project
#include "armor_solver/armor_solver_node.hpp"
#include "armor_solver/utils.hpp"

namespace rm_auto_aim {
Solver::Solver(std::weak_ptr<rclcpp::Node> n) : node_(n) {
  // 获取节点指针
  auto node = node_.lock();
  // 声明并获取参数
  shooting_range_w_ = node->declare_parameter("solver.shooting_range_width", 0.135);
  shooting_range_h_ = node->declare_parameter("solver.shooting_range_height", 0.135);
  max_tracking_v_yaw_ = node->declare_parameter("solver.max_tracking_v_yaw", 6.0);
  prediction_delay_ = node->declare_parameter("solver.prediction_delay", 0.0);
  controller_delay_ = node->declare_parameter("solver.controller_delay", 0.0);
  side_angle_ = node->declare_parameter("solver.side_angle", 15.0);
  min_switching_v_yaw_ = node->declare_parameter("solver.min_switching_v_yaw", 1.0);
  enable_ballistic_compensation_ =
    node->declare_parameter("solver.enable_ballistic_compensation", true);
  camera_pitch_extrinsic_ = node->declare_parameter("solver.camera_pitch_extrinsic", 0.0);
  gimbal_frame_ = node->declare_parameter("solver.gimbal_frame", std::string("gimbal_link"));
  tf_source_frame_ = node->declare_parameter("solver.tf_source_frame", std::string("odom"));

  // 声明并获取补偿器类型参数
  std::string compenstator_type = node->declare_parameter("solver.compensator_type", "ideal");
  // 根据补偿器类型创建补偿器
  trajectory_compensator_ = CompensatorFactory::createCompensator(compenstator_type);
  // 设置补偿器的迭代次数、速度、重力、阻力参数
  trajectory_compensator_->iteration_times = node->declare_parameter("solver.iteration_times", 20);
  trajectory_compensator_->velocity = node->declare_parameter("solver.bullet_speed", 20.0);
  trajectory_compensator_->gravity = node->declare_parameter("solver.gravity", 9.8);
  trajectory_compensator_->resistance = node->declare_parameter("solver.resistance", 0.001);

  // 创建手动补偿器
  manual_compensator_ = std::make_unique<ManualCompensator>();
  // 获取角度偏移参数
  auto angle_offset = node->declare_parameter("solver.angle_offset", std::vector<std::string>{});
  // 更新手动补偿器的地图流
  if(!manual_compensator_->updateMapFlow(angle_offset)) {
    RCLCPP_WARN(rclcpp::get_logger("armor_solver"), "Manual compensator update failed!");
  }

  // 设置初始状态为跟踪装甲板
  state = State::TRACKING_ARMOR;
  // 设置溢出计数器
  overflow_count_ = 0;
  // 设置转移阈值
  transfer_thresh_ = 5;

  R_target_gimbal_.setIdentity();
  t_target_gimbal_.setZero();

  // 释放节点指针
  node.reset();
}

auto_aim_interfaces::msg::GimbalCmd Solver::solve(const auto_aim_interfaces::msg::Target &target,
                                            const rclcpp::Time &current_time,
                                            std::shared_ptr<tf2_ros::Buffer> tf2_buffer_) {
  try {
    auto node = node_.lock();
    max_tracking_v_yaw_ = node->get_parameter("solver.max_tracking_v_yaw").as_double();
    prediction_delay_ = node->get_parameter("solver.prediction_delay").as_double();
    controller_delay_ = node->get_parameter("solver.controller_delay").as_double();
    side_angle_ = node->get_parameter("solver.side_angle").as_double();
    min_switching_v_yaw_ = node->get_parameter("solver.min_switching_v_yaw").as_double();
    enable_ballistic_compensation_ =
      node->get_parameter("solver.enable_ballistic_compensation").as_bool();
    camera_pitch_extrinsic_ = node->get_parameter("solver.camera_pitch_extrinsic").as_double();
    gimbal_frame_ = node->get_parameter("solver.gimbal_frame").as_string();
    tf_source_frame_ = node->get_parameter("solver.tf_source_frame").as_string();
    node.reset();
  } catch (const std::runtime_error &e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("armor_solver"), e.what());
  }

  const bool target_in_gimbal_frame = (target.header.frame_id == gimbal_frame_);

  try {
    auto gimbal_tf =
      tf2_buffer_->lookupTransform(tf_source_frame_, gimbal_frame_, tf2::TimePointZero);
    tf2::Quaternion tf_q;
    tf2::fromMsg(gimbal_tf.transform.rotation, tf_q);
    tf2::Matrix3x3(tf_q).getRPY(rpy_[0], rpy_[1], rpy_[2]);

    if (!target_in_gimbal_frame) {
      auto gimbal_in_target =
        tf2_buffer_->lookupTransform(target.header.frame_id, gimbal_frame_, tf2::TimePointZero);
      Eigen::Isometry3d iso = tf2::transformToEigen(gimbal_in_target);
      R_target_gimbal_ = iso.rotation();
      t_target_gimbal_ = iso.translation();
    } else {
      R_target_gimbal_.setIdentity();
      t_target_gimbal_.setZero();
    }
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("armor_solver"), ex.what());
    throw ex;
  }

  auto convertToGimbal = [&](const Eigen::Vector3d &p_target_frame) -> Eigen::Vector3d {
    if (target_in_gimbal_frame) {
      return p_target_frame;
    }
    const Eigen::Vector3d relative = p_target_frame - t_target_gimbal_;
    return R_target_gimbal_.transpose() * relative;
  };

  Eigen::Vector3d target_position(target.position.x, target.position.y, target.position.z);
  Eigen::Vector3d target_velocity(target.velocity.x, target.velocity.y, target.velocity.z);
  double target_yaw = target.yaw;
  double flying_time = 0.0;
  if (enable_ballistic_compensation_) {
    flying_time = trajectory_compensator_->getFlyingTime(convertToGimbal(target_position));
  }
  double dt =
    (current_time - rclcpp::Time(target.header.stamp)).seconds() + flying_time + prediction_delay_;
  target_position += target_velocity * dt;
  target_yaw += dt * target.v_yaw;

  std::vector<Eigen::Vector3d> armor_positions = getArmorPositions(
    target_position, target_yaw, target.radius_1, target.radius_2, target.dz, target.armors_num);
  int idx =
    selectBestArmor(armor_positions, target_position, target_yaw, target.v_yaw, target.armors_num);
  auto chosen_armor_position = armor_positions.at(idx);
  if (chosen_armor_position.norm() < 0.1) {
    throw std::runtime_error("No valid armor to shoot");
  }

  double raw_pitch = 0.0;
  double applied_pitch_offset = 0.0;

  auto_aim_interfaces::msg::GimbalCmd gimbal_cmd;
  gimbal_cmd.header = target.header;

  auto updateCommandFromVector = [&](const Eigen::Vector3d &position_target_frame) {
    const Eigen::Vector3d vec_in_gimbal = convertToGimbal(position_target_frame);
    const double distance = vec_in_gimbal.norm();

    double yaw_local = 0.0;
    double pitch_local = 0.0;
    calcYawAndPitch(vec_in_gimbal, yaw_local, pitch_local);

    const double yaw_abs = angles::normalize_angle(rpy_[2] + yaw_local);
    const double pitch_abs = rpy_[1] + pitch_local;

    const auto angle_offset = manual_compensator_->angleHardCorrect(
      vec_in_gimbal.head(2).norm(), vec_in_gimbal.z());
    const double pitch_offset = angle_offset[0] * M_PI / 180.0;
    const double yaw_offset = angle_offset[1] * M_PI / 180.0;

    const double cmd_pitch = pitch_abs + pitch_offset + camera_pitch_extrinsic_;
    const double cmd_yaw = angles::normalize_angle(yaw_abs + yaw_offset);

    raw_pitch = pitch_abs;
    applied_pitch_offset = pitch_offset;

    gimbal_cmd.distance = distance;
    gimbal_cmd.pitch = cmd_pitch;
    gimbal_cmd.yaw = cmd_yaw;
    gimbal_cmd.pitch_diff = cmd_pitch - rpy_[1];
    gimbal_cmd.yaw_diff = angles::normalize_angle(cmd_yaw - rpy_[2]);
    gimbal_cmd.fire_advice = isOnTarget(rpy_[2], rpy_[1], yaw_abs, pitch_abs, distance);

    RCLCPP_DEBUG(rclcpp::get_logger("armor_solver"),
                 "frame=%s vec=(%.3f,%.3f,%.3f) yaw_abs=%.4f pitch_abs=%.4f offsets(p=%.4f,y=%.4f)",
                 target.header.frame_id.c_str(), vec_in_gimbal.x(), vec_in_gimbal.y(),
                 vec_in_gimbal.z(), yaw_abs, pitch_abs, pitch_offset, yaw_offset);
  };

  updateCommandFromVector(chosen_armor_position);

  switch (state) {
    case TRACKING_ARMOR: {
      if (std::abs(target.v_yaw) > max_tracking_v_yaw_) {
        overflow_count_++;
      } else {
        overflow_count_ = 0;
      }

      if (overflow_count_ > transfer_thresh_) {
        state = TRACKING_CENTER;
      }

      if (!gimbal_cmd.fire_advice) {
        target_position += target_velocity * controller_delay_;
        target_yaw += controller_delay_ * target.v_yaw;
        armor_positions = getArmorPositions(target_position,
                                            target_yaw,
                                            target.radius_1,
                                            target.radius_2,
                                            target.dz,
                                            target.armors_num);
        chosen_armor_position = armor_positions.at(idx);
        if (chosen_armor_position.norm() < 0.1) {
          throw std::runtime_error("No valid armor to shoot");
        }
        updateCommandFromVector(chosen_armor_position);
      }
      break;
    }
    case TRACKING_CENTER: {
      if (std::abs(target.v_yaw) < max_tracking_v_yaw_) {
        overflow_count_++;
      } else {
        overflow_count_ = 0;
      }

      if (overflow_count_ > transfer_thresh_) {
        state = TRACKING_ARMOR;
        overflow_count_ = 0;
      }
      updateCommandFromVector(target_position);
      gimbal_cmd.fire_advice = true;
      break;
    }
  }

  RCLCPP_DEBUG(rclcpp::get_logger("armor_solver"),
               "pitch_dbg cur=%.4f raw=%.4f cmd=%.4f diff=%.4f offset=%.4f extr=%.4f dist=%.2f",
               rpy_[1], raw_pitch, gimbal_cmd.pitch, gimbal_cmd.pitch_diff, applied_pitch_offset,
               camera_pitch_extrinsic_, gimbal_cmd.distance);

  if (gimbal_cmd.fire_advice) {
    RCLCPP_DEBUG(rclcpp::get_logger("armor_solver"), "You Need Fire!");
  }

  return gimbal_cmd;
}

bool Solver::isOnTarget(const double cur_yaw,
                        const double cur_pitch,
                        const double target_yaw,
                        const double target_pitch,
                        const double distance) const noexcept {
  // Judge whether to shoot
  double shooting_range_yaw = std::abs(atan2(shooting_range_w_ / 2, distance));
  double shooting_range_pitch = std::abs(atan2(shooting_range_h_ / 2, distance));
  // Limit the shooting area to 1 degree to avoid not shooting when distance is
  // too large
  shooting_range_yaw = std::max(shooting_range_yaw, 1.0 * M_PI / 180);
  shooting_range_pitch = std::max(shooting_range_pitch, 1.0 * M_PI / 180);
  if (std::abs(cur_yaw - target_yaw) < shooting_range_yaw &&
      std::abs(cur_pitch - target_pitch) < shooting_range_pitch) {
    return true;
  }

  return false;
}

std::vector<Eigen::Vector3d> Solver::getArmorPositions(const Eigen::Vector3d &target_center,
                                                       const double target_yaw,
                                                       const double r1,
                                                       const double r2,
                                                       const double dz,
                                                       const size_t armors_num) const noexcept {
  auto armor_positions = std::vector<Eigen::Vector3d>(armors_num, Eigen::Vector3d::Zero());

  // 当前追踪的装甲板高度，另一个装甲板高度 = 当前高度 + dz（dz 可能为负）
  const double current_height = target_center.z();
  const double alternate_height = current_height + dz;

  bool use_current_pair = true;
  for (size_t i = 0; i < armors_num; i++) {
    const double temp_yaw = target_yaw + i * (2 * M_PI / armors_num);

    double radius = r1;
    double armor_height = current_height;

    if (armors_num == 4) {
      radius = use_current_pair ? r1 : r2;
      armor_height = use_current_pair ? current_height : alternate_height;
      use_current_pair = !use_current_pair;
    } else if (armors_num == 3) {
      // 前哨站装甲板高度一致，保留原始 r1 和 current_height
      radius = r1;
      armor_height = current_height;
    } else if (armors_num == 2) {
      // 平衡步兵只有两块装甲板且高度一致
      radius = r1;
      armor_height = current_height;
    }

    armor_positions[i] = Eigen::Vector3d(
      target_center.x() - radius * std::cos(temp_yaw),
      target_center.y() - radius * std::sin(temp_yaw),
      armor_height);
  }

  return armor_positions;
}

int Solver::selectBestArmor(const std::vector<Eigen::Vector3d> &armor_positions,
                            const Eigen::Vector3d &target_center,
                            const double target_yaw,
                            const double target_v_yaw,
                            const size_t armors_num) const noexcept {
  // Angle between the car's center and the X-axis
  double alpha = std::atan2(target_center.y(), target_center.x());
  // Angle between the front of observed armor and the X-axis
  double beta = target_yaw;

  // clang-format off
  Eigen::Matrix2d R_odom2center;
  Eigen::Matrix2d R_odom2armor;
  R_odom2center << std::cos(alpha), std::sin(alpha), 
                  -std::sin(alpha), std::cos(alpha);
  R_odom2armor << std::cos(beta), std::sin(beta), 
                 -std::sin(beta), std::cos(beta);
  // clang-format on
  Eigen::Matrix2d R_center2armor = R_odom2center.transpose() * R_odom2armor;

  // Equal to (alpha - beta) in most cases
  double decision_angle = -std::asin(R_center2armor(0, 1));

  // Angle thresh of the armor jump
  double theta = (target_v_yaw > 0 ? side_angle_ : -side_angle_) / 180.0 * M_PI;

  // Avoid the frequent switch between two armor
  if (std::abs(target_v_yaw) < min_switching_v_yaw_) {
    theta = 0;
  }

  double temp_angle = decision_angle + M_PI / armors_num - theta;

  if (temp_angle < 0) {
    temp_angle += 2 * M_PI;
  }

  int selected_id = static_cast<int>(temp_angle / (2 * M_PI / armors_num));
  return selected_id;
}

void Solver::calcYawAndPitch(const Eigen::Vector3d &p,
                             double &yaw,
                             double &pitch) const noexcept {
  // Calculate yaw and pitch
  yaw = atan2(p.y(), p.x());
  pitch = atan2(p.z(), p.head(2).norm());

  // If the trajectory compensator compensates the pitch, update the pitch
  if (enable_ballistic_compensation_) {
    double temp_pitch = pitch;
    if (trajectory_compensator_->compensate(p, temp_pitch)) {
      pitch = temp_pitch;
    }
  }
}

std::vector<std::pair<double, double>> Solver::getTrajectory() const noexcept {
  // 获取轨迹
  auto trajectory = trajectory_compensator_->getTrajectory(15, rpy_[1]);
  // Rotate
  for (auto &p : trajectory) {
    double x = p.first;
    double y = p.second;
    p.first = x * cos(rpy_[1]) + y * sin(rpy_[1]);
    p.second = -x * sin(rpy_[1]) + y * cos(rpy_[1]);
  }
  return trajectory;
}

}  // namespace rm_auto_aim
