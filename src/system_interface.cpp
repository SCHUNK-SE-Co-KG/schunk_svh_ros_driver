// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    system_interface.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2022/03/02
 *
 */
//-----------------------------------------------------------------------------

#include "schunk_svh_driver/system_interface.h"

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
#include <rclcpp/node.hpp>

namespace schunk_svh_driver
{
SystemInterface::return_type SystemInterface::configure(
  const hardware_interface::HardwareInfo & info)
{
  // Keep an internal copy of the given configuration
  if (configure_default(info) != return_type::OK) {
    return return_type::ERROR;
  }

  m_positions.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  m_velocities.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  m_currents.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  m_position_commands.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_ERROR(
        rclcpp::get_logger("SystemInterface"), "Joint '%s' needs a command interface.",
        joint.name.c_str());
      return return_type::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_ERROR(
        rclcpp::get_logger("SystemInterface"),
        "Joint '%s' needs a %s command interface.",
        joint.name.c_str(), hardware_interface::HW_IF_POSITION);
      return return_type::ERROR;
    }

    if (joint.state_interfaces.size() != 3) {
      RCLCPP_ERROR(
        rclcpp::get_logger("SystemInterface"), "Joint '%s' uses 3 state interfaces.",
        joint.name.c_str());
      return return_type::ERROR;
    }

    if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[1].name == hardware_interface::HW_IF_VELOCITY ||
          joint.state_interfaces[2].name == schunk_svh_driver::HW_IF_CURRENT)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("SystemInterface"),
        "Joint '%s' needs the following state interfaces in this order: %s, %s, and %s.",
        joint.name.c_str(), hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
        schunk_svh_driver::HW_IF_CURRENT);
      return return_type::ERROR;
    }
  }

  // Initialize ROS2 node
  m_node = std::make_unique<rclcpp::Node>(
    "schunk_svh_driver", rclcpp::NodeOptions().start_parameter_services(false).use_global_arguments(false));

  // Initialize SVH in parallel.
  // Detach the thread to die cleanly with the controller manager node.
  m_svh = std::make_unique<driver_svh::SVHFingerManager>();
  m_init_thread = std::thread(&SystemInterface::init, this);
  m_init_thread.detach();

  this->status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
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
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  // We currently allow any combination of command interfaces.
  return return_type::OK;
}

SystemInterface::return_type SystemInterface::start()
{
  this->status_ = hardware_interface::status::STARTED;
  RCLCPP_INFO(rclcpp::get_logger("SystemInterface"), "Started SVH driver");
  return return_type::OK;
}

SystemInterface::return_type SystemInterface::stop()
{
  m_svh->disconnect();

  this->status_ = hardware_interface::status::STOPPED;
  RCLCPP_INFO(rclcpp::get_logger("SystemInterface"), "Stopped SVH driver");
  return return_type::OK;
}

SystemInterface::return_type SystemInterface::read()
{
  if (m_svh->isConnected())
  {
    for (size_t channel = 0; channel < driver_svh::eSVH_DIMENSION; ++channel)
    {
      if (m_svh->isHomed(static_cast<driver_svh::SVHChannel>(channel)))  // resetted and ready to use
      {
        m_svh->getPosition(static_cast<driver_svh::SVHChannel>(channel), m_positions[channel]);
        m_svh->getCurrent(static_cast<driver_svh::SVHChannel>(channel), m_currents[channel]);
      }
    }
  }
  return return_type::OK;
}

SystemInterface::return_type SystemInterface::write() { return return_type::OK; }


void SystemInterface::init()
{
  auto firmware = m_svh->getFirmwareInfo("/dev/ttyUSB0");
  if (!m_svh->connect("/dev/ttyUSB0"))
  {
    RCLCPP_ERROR(rclcpp::get_logger("SystemInterface"), "No connection to the Schunk SVH");
    return;
  }

  if (!m_svh->resetChannel(driver_svh::eSVH_ALL))
  {
    RCLCPP_ERROR(rclcpp::get_logger("SystemInterface"), "Could not reset the Schunk SVH");
    return;
  }
}

}  // namespace schunk_svh_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(schunk_svh_driver::SystemInterface, hardware_interface::SystemInterface)
