////////////////////////////////////////////////////////////////////////////////
//
// © Copyright 2022 SCHUNK Mobile Greifsysteme GmbH, Lauffen/Neckar Germany
// © Copyright 2022 FZI Forschungszentrum Informatik, Karlsruhe, Germany
//
// This file is part of the Schunk SVH Driver.
//
// The Schunk SVH Driver is free software: you can redistribute it and/or
// modify it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.
//
// The Schunk SVH Driver is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
// Public License for more details.
//
// You should have received a copy of the GNU General Public License along with
// the Schunk SVH Driver. If not, see <https://www.gnu.org/licenses/>.
//
////////////////////////////////////////////////////////////////////////////////

//-----------------------------------------------------------------------------
/*!\file    system_interface.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2022/03/02
 *
 */
//-----------------------------------------------------------------------------

#include "schunk_svh_driver/system_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "schunk_svh_driver/ROSLogHandler.hpp"
#include "schunk_svh_library/LogLevel.h"
#include "schunk_svh_library/control/SVHCurrentSettings.h"
#include "schunk_svh_library/control/SVHHomeSettings.h"
#include "schunk_svh_library/control/SVHPositionSettings.h"

namespace schunk_svh_driver
{
SystemInterface::CallbackReturn SystemInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // Keep an internal copy of the given configuration
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    SystemInterface::CallbackReturn::SUCCESS) {
    return SystemInterface::CallbackReturn::ERROR;
  }

  m_device_file = info_.hardware_parameters["device_file"];

  m_positions.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  m_velocities.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  m_efforts.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  m_currents.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  m_position_commands.resize(info_.joints.size(), 0.0);

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_ERROR(
        rclcpp::get_logger("SystemInterface"), "Joint '%s' needs a command interface.",
        joint.name.c_str());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_ERROR(
        rclcpp::get_logger("SystemInterface"), "Joint '%s' needs a %s command interface.",
        joint.name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 4) {
      RCLCPP_ERROR(
        rclcpp::get_logger("SystemInterface"), "Joint '%s' uses 4 state interfaces.",
        joint.name.c_str());
      return CallbackReturn::ERROR;
    }

    if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[1].name == hardware_interface::HW_IF_VELOCITY ||
          joint.state_interfaces[2].name == hardware_interface::HW_IF_EFFORT ||
          joint.state_interfaces[3].name == schunk_svh_driver::HW_IF_CURRENT)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("SystemInterface"),
        "Joint '%s' needs the following state interfaces in this order: %s, %s, %s, and %s.",
        joint.name.c_str(), hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
        hardware_interface::HW_IF_EFFORT, schunk_svh_driver::HW_IF_CURRENT);
      return CallbackReturn::ERROR;
    }
  }

  // Activate logging in the library
  driver_svh::setupROSLogHandler();

  // Initialize SVH in parallel.
  // Detach the thread to die cleanly with the controller manager node.
  m_svh = std::make_unique<driver_svh::SVHFingerManager>();
  m_init_thread = std::thread(&SystemInterface::init, this);
  m_init_thread.detach();

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &m_positions[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &m_velocities[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &m_efforts[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, schunk_svh_driver::HW_IF_CURRENT, &m_currents[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &m_position_commands[i]));
  }

  return command_interfaces;
}

SystemInterface::return_type SystemInterface::prepare_command_mode_switch(
  [[maybe_unused]] const std::vector<std::string> & start_interfaces,
  [[maybe_unused]] const std::vector<std::string> & stop_interfaces)
{
  // We currently allow any combination of command interfaces.
  return return_type::OK;
}

SystemInterface::CallbackReturn SystemInterface::on_activate(
  [[maybe_unused]] const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("SystemInterface"), "Started SVH driver");
  return CallbackReturn::SUCCESS;
}

SystemInterface::CallbackReturn SystemInterface::on_shutdown(
  [[maybe_unused]] const rclcpp_lifecycle::State & previous_state)
{
  m_svh->disconnect();

  RCLCPP_INFO(rclcpp::get_logger("SystemInterface"), "Stopped SVH driver");
  return CallbackReturn::SUCCESS;
}

SystemInterface::return_type SystemInterface::read(
  [[maybe_unused]] const rclcpp::Time & time, [[maybe_unused]] const rclcpp::Duration & period)
{
  if (m_svh->isConnected()) {
    for (size_t channel = 0; channel < driver_svh::SVH_DIMENSION; ++channel) {
      if (m_svh->isHomed(
            static_cast<driver_svh::SVHChannel>(channel)))  // resetted and ready to use
      {
        m_svh->getPosition(static_cast<driver_svh::SVHChannel>(channel), m_positions[channel]);
        m_svh->getCurrent(static_cast<driver_svh::SVHChannel>(channel), m_currents[channel]);

        // Joint efforts are an estimation based on motor currents
        m_svh->getCurrent(static_cast<driver_svh::SVHChannel>(channel), m_efforts[channel]);
        m_efforts[channel] =
          m_svh->convertmAtoN(static_cast<driver_svh::SVHChannel>(channel), m_efforts[channel]);
      }
    }
  }

  return return_type::OK;
}

SystemInterface::return_type SystemInterface::write(
  [[maybe_unused]] const rclcpp::Time & time, [[maybe_unused]] const rclcpp::Duration & period)
{
  if (m_initialized) {
    m_svh->setAllTargetPositions(m_position_commands);  // Does all plausibility checks
  }
  return return_type::OK;
}

void SystemInterface::init()
{
  if (!m_svh->connect(m_device_file)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SystemInterface"), "No connection to the Schunk SVH under %s",
      m_device_file.c_str());
    return;
  }
  auto firmware = m_svh->getFirmwareInfo(m_device_file);
  auto version =
    std::to_string(firmware.version_major) + "." + std::to_string(firmware.version_minor) + ".";
  RCLCPP_INFO(
    rclcpp::get_logger("SystemInterface"), "The Schunk SVH is version: %s", version.c_str());

  // Convert std::string("1.0 2.0") to std::vector<float>{1.0, 2.0}
  auto make_floats = [](const std::string & s) {
    std::vector<float> vec;
    std::stringstream stream(s);
    std::string element;

    while (std::getline(stream, element, ' ')) {
      vec.emplace_back(std::stof(element));
    }
    return vec;
  };

  for (size_t i = 0; i < driver_svh::SVH_DIMENSION; ++i) {
    auto current_settings = info_.joints[i].parameters[version + "current_controller"];
    auto position_settings = info_.joints[i].parameters[version + "position_controller"];
    auto home_settings = info_.joints[i].parameters[version + "home_settings"];

    // Use default values if the current version is not directly supported.
    if (current_settings.empty()) {
      current_settings = info_.joints[i].parameters["0.0.current_controller"];
      RCLCPP_WARN(
        rclcpp::get_logger("SystemInterface"),
        "Channel %zu parameters for motor currents not available. Using defaults.", i);
    }
    if (position_settings.empty()) {
      position_settings = info_.joints[i].parameters["0.0.position_controller"];
      RCLCPP_WARN(
        rclcpp::get_logger("SystemInterface"),
        "Channel %zu parameters for position controllers not available. Using defaults.", i);
    }
    if (home_settings.empty()) {
      home_settings = info_.joints[i].parameters["0.0.home_settings"];
      RCLCPP_WARN(
        rclcpp::get_logger("SystemInterface"),
        "Channel %zu parameters for home settings not available. Using defaults.", i);
    }

    m_svh->setCurrentSettings(
      static_cast<driver_svh::SVHChannel>(i),
      driver_svh::SVHCurrentSettings(make_floats(current_settings)));

    m_svh->setPositionSettings(
      static_cast<driver_svh::SVHChannel>(i),
      driver_svh::SVHPositionSettings(make_floats(position_settings)));

    m_svh->setHomeSettings(
      static_cast<driver_svh::SVHChannel>(i),
      driver_svh::SVHHomeSettings(make_floats(home_settings)));
  }

  m_initialized = m_svh->resetChannel(driver_svh::SVH_ALL);
  if (!m_initialized) {
    RCLCPP_ERROR(rclcpp::get_logger("SystemInterface"), "Could not initialize the Schunk SVH");
    return;
  }
}

}  // namespace schunk_svh_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(schunk_svh_driver::SystemInterface, hardware_interface::SystemInterface)
