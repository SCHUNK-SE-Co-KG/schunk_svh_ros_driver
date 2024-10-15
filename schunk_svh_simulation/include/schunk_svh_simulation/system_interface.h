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
/*!\file    system_interface.h
 *
 * \author  Stefan Scherzinger <stefan.scherzinger@de.schunk.com>
 * \date    2024/10/01
 *
 */
//-----------------------------------------------------------------------------

#pragma once

#include <map>
#include <thread>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "schunk_svh_simulation/mujoco_simulator.h"
#include "std_msgs/msg/bool.hpp"

namespace schunk_svh_simulation
{

// Additional hardware interfaces for the SVH:
constexpr char HW_IF_CURRENT[] = "current";

/**
 * @brief A MuJoCo-based, standalone simulator for the SCHUNK SVH and ROS2-control
 *
 * This class provides a simulated robot for controller development and
 * testing.  It's instantiated via the usual ROS2-conform lifecylce as a
 * controller_manager coordinated library.
 *
 */
class Simulator : public hardware_interface::SystemInterface
{
public:
  using return_type = hardware_interface::return_type;
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  RCLCPP_SHARED_PTR_DEFINITIONS(Simulator)

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Command buffers for the controllers
  std::vector<double> m_position_commands;

  // State buffers for the controllers
  std::vector<double> m_positions;
  std::vector<double> m_velocities;
  std::vector<double> m_efforts;
  std::vector<double> m_currents;

  // Run MuJoCo's solver in a separate thread
  std::thread m_simulation;

  // Make this a ROS2 node
  std::shared_ptr<rclcpp::Node> m_node;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_svh_initialized_publisher;

  // Parameters
  std::string m_mujoco_model;
  std::string m_mesh_dir;
};

}  // namespace schunk_svh_simulation
