// Copyright 2021 ros2_control Development Team
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

#include "reasonable_outdoor_mobile_robot_hardware/reasonable_outdoor_mobile_robot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace reasonable_outdoor_mobile_robot_hardware
{
CallbackReturn ReasonableRobotSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  device_name_ = "/dev/ttyACM0";
  device_name_ = info_.hardware_parameters["device_name"];
  
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // BlvDiffbotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ReasonableRobotSystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ReasonableRobotSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 3)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ReasonableRobotSystemHardware"),
        "Joint '%s' has %zu state interface. 3 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ReasonableRobotSystemHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ReasonableRobotSystemHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ReasonableRobotSystemHardware"),
        "Joint '%s' have '%s' as third state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_EFFORT);
      return CallbackReturn::ERROR;
    }
  }

  serial_port_ = std::make_shared<ReasonableRobotArduinoComunicator>(device_name_, info_.joints.size());
  
  prev_pos_.resize(info_.joints.size());
  for (int i = 0; i < info_.joints.size(); i++)
  {
    prev_pos_[i] = 0.0f;
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ReasonableRobotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ReasonableRobotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

CallbackReturn ReasonableRobotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ReasonableRobotSystemHardware"), "Starting ...please wait...");

  for (auto i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("ReasonableRobotSystemHardware"), "%.1f seconds left...", hw_start_sec_ - i);
  }

  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }
  
  if (!serial_port_->isConnected()) {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("ReasonableRobotSystemHardware"), "System Successfully started!");

  return CallbackReturn::SUCCESS;
}

CallbackReturn ReasonableRobotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ReasonableRobotSystemHardware"), "Stopping ...please wait...");

  for (auto i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("ReasonableRobotSystemHardware"), "%.1f seconds left...", hw_stop_sec_ - i);
  }
  
  if (serial_port_->isConnected()) {
    //serial_port_->closeDevice();
  }

  RCLCPP_INFO(rclcpp::get_logger("ReasonableRobotSystemHardware"), "System successfully stopped!");

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type ReasonableRobotSystemHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{

  std::vector<float> rad(2);
  std::vector<float> radps(2);
  std::vector<float> current(2);
  serial_port_->readRad(rad, radps, current);

  int motor_direction;
  for (uint i = 0; i < hw_commands_.size(); i++)
  {
    if (i % 2 == 0)
    {
      motor_direction = 1;
    }
    else
    {
      motor_direction = -1;
    }

    if (prev_pos_[i] == 0.0f)
    {
      prev_pos_[i] = rad[i];
    }
    else
    {
      float diff_rad = rad[i] - prev_pos_[i];
      if (diff_rad > M_PI)
      {
        diff_rad -= 2*M_PI;
      }
      if (diff_rad < -M_PI)
      {
        diff_rad += 2*M_PI;
      }
      prev_pos_[i] = rad[i];
      hw_positions_[i] += static_cast<double>(motor_direction * diff_rad);
    }
    hw_velocities_[i] = static_cast<double>(motor_direction * radps[i]);

    hw_efforts_[i] = static_cast<double>(motor_direction * current[i]);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ReasonableRobotSystemHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  std::vector<float> send_commands_radps(hw_commands_.size());
  int motor_direction;
  for (auto i = 0u; i < hw_commands_.size(); i++)
  {
    if (i % 2 == 0)
    {
      motor_direction = 1;
    }
    else
    {
      motor_direction = -1;
    }

    // Generate the motor command message
    send_commands_radps[i] = motor_direction * hw_commands_[i];
  }
  serial_port_->writeRadps(send_commands_radps);
  
  return hardware_interface::return_type::OK;
}

}  // namespace reasonable_outdoor_mobile_robot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  reasonable_outdoor_mobile_robot_hardware::ReasonableRobotSystemHardware, hardware_interface::SystemInterface)
