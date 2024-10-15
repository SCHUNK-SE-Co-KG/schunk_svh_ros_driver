// Copyright 2024 SCHUNK SE & Co. KG
//
// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along with
// this program. If not, see <https://www.gnu.org/licenses/>.
// --------------------------------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    system_interface.cpp
 *
 * \author  Stefan Scherzinger <stefan.scherzinger@de.schunk.com>
 * \date    2024/10/01
 *
 */
//-----------------------------------------------------------------------------

#include "schunk_svh_simulation/system_interface.h"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "schunk_svh_simulation/mujoco_simulator.h"

namespace schunk_svh_simulation
{

Simulator::CallbackReturn Simulator::on_init(const hardware_interface::HardwareInfo & info)
{
  // Keep an internal copy of the given configuration
  if (hardware_interface::SystemInterface::on_init(info) != Simulator::CallbackReturn::SUCCESS) {
    return Simulator::CallbackReturn::ERROR;
  }

  // Start the simulator in parallel.
  // Let the thread's destructor clean-up all resources
  // once users close the simulation window.
  m_mujoco_model = info_.hardware_parameters["mujoco_model"];
  m_mesh_dir = info_.hardware_parameters["mesh_directory"];
  m_simulation = std::thread(MuJoCoSimulator::simulate, m_mujoco_model, m_mesh_dir);
  m_simulation.detach();

  m_positions.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  m_velocities.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  m_efforts.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  m_currents.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  m_position_commands.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_ERROR(
        rclcpp::get_logger("Simulator"), "Joint '%s' needs a command interface.",
        joint.name.c_str());

      return Simulator::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_ERROR(
        rclcpp::get_logger("Simulator"), "Joint '%s' needs the following command interfaces: %s",
        joint.name.c_str(), hardware_interface::HW_IF_POSITION);

      return Simulator::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 4) {
      RCLCPP_ERROR(
        rclcpp::get_logger("Simulator"), "Joint '%s' needs 4 state interfaces.",
        joint.name.c_str());

      return Simulator::CallbackReturn::ERROR;
    }

    if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[1].name == hardware_interface::HW_IF_VELOCITY ||
          joint.state_interfaces[2].name == hardware_interface::HW_IF_EFFORT ||
          joint.state_interfaces[3].name == schunk_svh_simulation::HW_IF_CURRENT)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("Simulator"),
        "Joint '%s' needs the following state interfaces in that order: %s, %s, %s, and %s.",
        joint.name.c_str(), hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
        hardware_interface::HW_IF_EFFORT, schunk_svh_simulation::HW_IF_CURRENT);

      return Simulator::CallbackReturn::ERROR;
    }
  }

  // Feedback on initialization
  m_node = std::make_shared<rclcpp::Node>("svh", rclcpp::NodeOptions());
  m_svh_initialized_publisher = m_node->create_publisher<std_msgs::msg::Bool>(
    m_node->get_name() + std::string("/initialized"), 3);

  auto msg = std_msgs::msg::Bool();
  msg.data = true;
  m_svh_initialized_publisher->publish(msg);
  return Simulator::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Simulator::export_state_interfaces()
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
      info_.joints[i].name, schunk_svh_simulation::HW_IF_CURRENT, &m_currents[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Simulator::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &m_position_commands[i]));
  }

  return command_interfaces;
}

Simulator::return_type Simulator::prepare_command_mode_switch(
  [[maybe_unused]] const std::vector<std::string> & start_interfaces,
  [[maybe_unused]] const std::vector<std::string> & stop_interfaces)
{
  // TODO: Exclusive OR for position and velocity commands

  return return_type::OK;
}

Simulator::return_type Simulator::read(
  [[maybe_unused]] const rclcpp::Time & time, [[maybe_unused]] const rclcpp::Duration & period)
{
  MuJoCoSimulator::getInstance().read(m_positions, m_velocities, m_efforts, m_currents);

  // Start with the current positions as safe default, but let active
  // controllers overrride them in each cycle.
  if (std::any_of(m_position_commands.begin(), m_position_commands.end(), [](double i) {
        return std::isnan(i);
      })) {
    m_position_commands = m_positions;
  }

  MuJoCoSimulator::getInstance().read(m_positions, m_velocities, m_efforts, m_currents);
  m_positions = m_position_commands;  // Open-loop control

  return return_type::OK;
}

Simulator::return_type Simulator::write(
  [[maybe_unused]] const rclcpp::Time & time, [[maybe_unused]] const rclcpp::Duration & period)
{
  MuJoCoSimulator::getInstance().write(m_position_commands);
  return return_type::OK;
}

}  // namespace schunk_svh_simulation

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(schunk_svh_simulation::Simulator, hardware_interface::SystemInterface)
