// Copyright 2025 Enactic, Inc.
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

#include "openarm_hardware/openarm_simple_hardware.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

namespace openarm_hardware {

OpenArmHW::OpenArmHW() = default;

bool OpenArmHW::parse_config(const hardware_interface::HardwareInfo& info) {
  // Parse CAN interface (default: can0)
  auto it = info.hardware_parameters.find("can_interface");
  can_interface_ = (it != info.hardware_parameters.end()) ? it->second : "can0";

  // Parse arm prefix (default: empty for single arm, "left_" or "right_" for
  // bimanual)
  it = info.hardware_parameters.find("arm_prefix");
  arm_prefix_ = (it != info.hardware_parameters.end()) ? it->second : "";

  it = info.hardware_parameters.find("single_motor_test");
  if (it != info.hardware_parameters.end()) {
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    single_motor_test_ = value == "true";
  }

  // Parse gripper enable (default: true for V10)
  it = info.hardware_parameters.find("hand");
  if (it == info.hardware_parameters.end()) {
    hand_ = true;  // Default to true for V10
  } else {
    // Handle both "true"/"True" and "false"/"False"
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    hand_ = (value == "true");
  }

  // Parse CAN-FD enable (default: true for V10)
  it = info.hardware_parameters.find("can_fd");
  if (it == info.hardware_parameters.end()) {
    can_fd_ = true;  // Default to true for V10
  } else {
    // Handle both "true"/"True" and "false"/"False"
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    can_fd_ = (value == "true");
  }
  if (!can_fd_) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"),
                 "ZK motor protocol requires CAN FD");
    return false;
  }

  // Parse control gains
  for (size_t i = 1; i <= ARM_DOF; ++i) {
    it = info.hardware_parameters.find("kp" + std::to_string(i));
    if (it != info.hardware_parameters.end()) {
      kp_[i - 1] = std::stod(it->second);
    }
    it = info.hardware_parameters.find("kd" + std::to_string(i));
    if (it != info.hardware_parameters.end()) {
      kd_[i - 1] = std::stod(it->second);
    }
  }
  if (single_motor_test_ &&
      (!std::isfinite(kp_[0]) || !std::isfinite(kd_[0]) || kp_[0] < 0.0 ||
       kd_[0] < 0.0)) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"),
                 "Single-motor kp and kd must be non-negative");
    return false;
  }
  // Parse ee_type (default: parallel_link for v10)
  it = info.hardware_parameters.find("ee_type");
  ee_type_ =
      (it != info.hardware_parameters.end()) ? it->second : "parallel_link";
  if (hand_) {
    it = info.hardware_parameters.find("kp_hand");
    if (it != info.hardware_parameters.end()) {
      gripper_kp_ = std::stod(it->second);
    }
    it = info.hardware_parameters.find("kd_hand");
    if (it != info.hardware_parameters.end()) {
      gripper_kd_ = std::stod(it->second);
    }
  }

  RCLCPP_INFO(
      rclcpp::get_logger("OpenArmHW"),
      "Configuration: CAN=%s, arm_prefix=%s, hand=%s, can_fd=%s, single_motor_test=%s",
      can_interface_.c_str(), arm_prefix_.c_str(),
      hand_ ? "enabled" : "disabled", can_fd_ ? "enabled" : "disabled",
      single_motor_test_ ? "true" : "false");
  return true;
}

void OpenArmHW::generate_joint_names() {
  joint_names_.clear();
  // TODO: read from urdf properly and sort in the future.
  // Currently, the joint names are hardcoded for order consistency to align
  // with hardware. Generate arm joint names: openarm_{arm_prefix}joint{N}
  for (size_t i = 1; i <= ARM_DOF; ++i) {
    std::string joint_name =
        "openarm_" + arm_prefix_ + "joint" + std::to_string(i);
    joint_names_.push_back(joint_name);
  }

  // Generate gripper joint name if enabled
  if (hand_) {
    std::string gripper_joint_name = "openarm_" + arm_prefix_ + "finger_joint1";
    joint_names_.push_back(gripper_joint_name);
    RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "Added gripper joint: %s",
                gripper_joint_name.c_str());
  } else {
    RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"),
                "Gripper joint NOT added because hand_=false");
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"),
              "Generated %zu joint names for arm prefix '%s'",
              joint_names_.size(), arm_prefix_.c_str());
}

hardware_interface::CallbackReturn OpenArmHW::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  // Parse configuration
  if (!parse_config(info)) {
    return CallbackReturn::ERROR;
  }

  // Generate joint names based on arm prefix
  generate_joint_names();

  // Validate joint count (7 arm joints + optional gripper)
  size_t expected_joints = ARM_DOF + (hand_ ? 1 : 0);
  if (joint_names_.size() != expected_joints) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"),
                 "Generated %zu joint names, expected %zu", joint_names_.size(),
                 expected_joints);
    return CallbackReturn::ERROR;
  }

  // Test mode exposes the full arm to ros2_control but only opens ZK motor 1.
  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"),
              "Initializing OpenArm ZK motors on %s...", can_interface_.c_str());
  std::vector<uint8_t> motor_ids =
      single_motor_test_ ? std::vector<uint8_t>{1}
                         : std::vector<uint8_t>{1, 2, 3, 4, 5, 6, 7};
  if (hand_ && !single_motor_test_) motor_ids.push_back(8);
  openarm_ = std::make_unique<openarm::can::socket::ZKOpenArm>(can_interface_,
                                                               motor_ids);

  // Initialize state and command vectors based on generated joint count
  const size_t total_joints = joint_names_.size();
  pos_commands_.resize(total_joints, 0.0);
  vel_commands_.resize(total_joints, 0.0);
  tau_commands_.resize(total_joints, 0.0);
  pos_states_.resize(total_joints, 0.0);
  vel_states_.resize(total_joints, 0.0);
  tau_states_.resize(total_joints, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"),
              "OpenArm V10 Simple HW initialized successfully");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArmHW::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Set callback mode to ignore during configuration
  openarm_->refresh_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all(5000);

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
OpenArmHW::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
OpenArmHW::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  // TODO: consider exposing only needed interfaces to avoid undefined behavior.
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION,
        &pos_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY,
        &vel_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn OpenArmHW::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  single_motor_controller_active_ = false;
  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "Activating OpenArm V10...");
  openarm_->enable_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all(5000);

  if (single_motor_test_) {
    openarm_->refresh_all();
    openarm_->recv_all(5000);
    if (openarm_->last_received_count() != 1) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"),
                   "Single-motor test received %zu/1 motor states",
                   openarm_->last_received_count());
      openarm_->disable_all();
      return CallbackReturn::ERROR;
    }
    const auto motor = openarm_->get_motors().front();
    pos_states_[0] = motor.get_position();
    vel_states_[0] = motor.get_velocity();
    tau_states_[0] = motor.get_torque();
    pos_commands_[0] = pos_states_[0];
  } else {
    return_to_zero();
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "OpenArm V10 activated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArmHW::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  single_motor_controller_active_ = false;
  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "Deactivating OpenArm V10...");

  // Disable all motors (like full_arm.cpp exit)
  for (int i = 0; i < 3; ++i) {
    openarm_->disable_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    openarm_->recv_all(5000);
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "OpenArm V10 deactivated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type OpenArmHW::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  if (!single_motor_test_) {
    return hardware_interface::return_type::OK;
  }
  const std::string position_interface = joint_names_.front() + "/position";
  if (std::find(stop_interfaces.begin(), stop_interfaces.end(),
                position_interface) != stop_interfaces.end()) {
    single_motor_controller_active_ = false;
  }
  if (std::find(start_interfaces.begin(), start_interfaces.end(),
                position_interface) != start_interfaces.end()) {
    single_motor_controller_active_ = true;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArmHW::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  const auto motors = openarm_->get_motors();
  const size_t real_motor_count = single_motor_test_ ? 1 : ARM_DOF;
  for (size_t i = 0; i < real_motor_count && i < motors.size(); ++i) {
    pos_states_[i] = motors[i].get_position();
    vel_states_[i] = motors[i].get_velocity();
    tau_states_[i] = motors[i].get_torque();
  }

  if (single_motor_test_) {
    for (size_t i = 1; i < joint_names_.size(); ++i) {
      pos_states_[i] = pos_commands_[i];
      vel_states_[i] = 0.0;
      tau_states_[i] = 0.0;
    }
    return hardware_interface::return_type::OK;
  }

  // Read gripper state if enabled
  if (hand_ && joint_names_.size() > ARM_DOF) {
    if (motors.size() > ARM_DOF) {
      // TODO the mappings are approximates
      // Convert motor position (radians) to joint value (0-0.044m)
      double motor_pos = motors[ARM_DOF].get_position();
      pos_states_[ARM_DOF] = motor_radians_to_joint(motor_pos);

      // Unimplemented: Velocity and torque mapping
      vel_states_[ARM_DOF] = 0;  // gripper_motors[0].get_velocity();
      tau_states_[ARM_DOF] = 0;  // gripper_motors[0].get_torque();
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArmHW::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Control arm motors with MIT control
  std::vector<openarm::damiao_motor::MITParam> motor_params;
  const size_t real_motor_count = single_motor_test_ ? 1 : ARM_DOF;
  for (size_t i = 0; i < real_motor_count; ++i) {
    const bool hold_position =
        single_motor_test_ && !single_motor_controller_active_;
    motor_params.push_back(
        {kp_[i], kd_[i], hold_position ? pos_states_[i] : pos_commands_[i],
         hold_position ? 0.0 : vel_commands_[i],
         hold_position ? 0.0 : tau_commands_[i]});
  }
  if (hand_ && !single_motor_test_ && joint_names_.size() > ARM_DOF) {
    motor_params.push_back({gripper_kp_, gripper_kd_,
                            joint_to_motor_radians(pos_commands_[ARM_DOF]),
                            0.0, 0.0});
  }
  openarm_->get_arm().mit_control_all(motor_params);
  openarm_->recv_all(single_motor_test_ ? 5000 : 1000);
  const size_t expected_motor_count =
      single_motor_test_ ? 1 : ARM_DOF + (hand_ ? 1 : 0);
  if (openarm_->last_received_count() != expected_motor_count) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"),
                 "Received %zu/%zu ZK motor states",
                 openarm_->last_received_count(), expected_motor_count);
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

void OpenArmHW::return_to_zero() {
  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "Returning to zero position...");

  openarm_->refresh_all();
  // Return arm to zero with MIT control
  std::vector<openarm::damiao_motor::MITParam> motor_params;
  for (size_t i = 0; i < ARM_DOF; ++i) {
    motor_params.push_back({kp_[i], kd_[i], 0.0, 0.0, 0.0});
  }
  if (hand_) {
    motor_params.push_back({gripper_kp_, gripper_kd_,
                            joint_to_motor_radians(GRIPPER_JOINT_0_POSITION),
                            0.0, 0.0});
  }
  openarm_->get_arm().mit_control_all(motor_params);
  std::this_thread::sleep_for(std::chrono::microseconds(1000));
  openarm_->recv_all(5000);
  const auto arm_motors = openarm_->get_arm().get_motors();

  std::vector<double> start_pos(ARM_DOF, 0.0);
  for (size_t i = 0; i < ARM_DOF && i < arm_motors.size(); ++i) {
    start_pos[i] = arm_motors[i].get_position();
  }

  const int steps = 200;
  const int step_ms = 10;

  for (int step = 0; step <= steps; ++step) {
    double t = static_cast<double>(step) / steps;  // 0.0 → 1.0

    std::vector<openarm::damiao_motor::MITParam> motor_params;
    for (size_t i = 0; i < ARM_DOF; ++i) {
      double target = start_pos[i] + t * (ZERO_POSITION[i] - start_pos[i]);
      motor_params.push_back({kp_[i], kd_[i], target, 0.0, 0.0});
    }
    if (hand_) {
      motor_params.push_back({gripper_kp_, gripper_kd_,
                              joint_to_motor_radians(GRIPPER_JOINT_0_POSITION),
                              0.0, 0.0});
    }
    openarm_->get_arm().mit_control_all(motor_params);

    openarm_->recv_all(1000);
    std::this_thread::sleep_for(std::chrono::milliseconds(step_ms));
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "Reached zero position");
}

double OpenArmHW::joint_to_motor_radians(double joint_value) {
  if (ee_type_ == "pinch_gripper") {
    // revolute: joint 0-1.5708 rad -> motor 0-1.5708
    return joint_value;
  } else {
    // parallel_link (prismatic): 0-0.044m -> 0 to -1.0472 rad
    return (joint_value / GRIPPER_JOINT_0_POSITION) * GRIPPER_MOTOR_1_RADIANS;
  }
}

double OpenArmHW::motor_radians_to_joint(double motor_radians) {
  if (ee_type_ == "pinch_gripper") {
    // revolute:
    return motor_radians;
  } else {
    // parallel_link (prismatic)
    return GRIPPER_JOINT_0_POSITION * (motor_radians / GRIPPER_MOTOR_1_RADIANS);
  }
}

// // Gripper mapping helper functions
// double OpenArmHW::joint_to_motor_radians(double joint_value) {
//   // Joint 0=closed -> motor 0 rad, Joint 0.044=open -> motor -1.0472 rad
//   return (joint_value / GRIPPER_JOINT_0_POSITION) *
//          GRIPPER_MOTOR_1_RADIANS;  // Scale from 0-0.044 to 0 to -1.0472
// }

// double OpenArmHW::motor_radians_to_joint(double motor_radians) {
//   // Motor 0 rad=closed -> joint 0, Motor -1.0472 rad=open -> joint 0.044
//   return GRIPPER_JOINT_0_POSITION *
//          (motor_radians /
//           GRIPPER_MOTOR_1_RADIANS);  // Scale from 0 to -1.0472 to 0-0.044
// }

}  // namespace openarm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(openarm_hardware::OpenArmHW,
                       hardware_interface::SystemInterface)
