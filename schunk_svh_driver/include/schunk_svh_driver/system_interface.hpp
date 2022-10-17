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
/*!\file    system_interface.hpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2022/03/02
 *
 */
//-----------------------------------------------------------------------------

#pragma once

#include <map>
#include <memory>
#include <thread>

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "schunk_svh_library/control/SVHFingerManager.h"

namespace schunk_svh_driver
{
// Additional hardware interfaces for the SVH:
constexpr char HW_IF_CURRENT[] = "current";

/**
   * @brief A ROS2-control SystemInterface for the Schunk SVH
   *
   * This class provides the hardware abstraction for the Schunk SVH in ROS2.
   * It's instantiated via the usual ROS2-conform lifecylce as a
   * controller_manager coordinated library.  Its goal is to support all
   * relevant hardware_interfaces as exposed by the schunk_svh_library.
   *
   */
class SystemInterface
: public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  using return_type = hardware_interface::return_type;

  RCLCPP_SHARED_PTR_DEFINITIONS(SystemInterface)

  return_type configure(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  return_type start() override;

  return_type stop() override;

  return_type read() override;

  return_type write() override;

private:
  // Initialization of the SVH in a separate thread to meet ROS2-control's
  // timing requirements.
  void init();
  std::atomic<bool> m_initialized{false};
  std::thread m_init_thread;

  // Handle to the SVH driver library
  std::unique_ptr<driver_svh::SVHFingerManager> m_svh;

  // Command buffers for the controllers
  std::vector<double> m_position_commands;

  // State buffers for the controllers
  std::vector<double> m_positions;
  std::vector<double> m_velocities;
  std::vector<double> m_efforts;
  std::vector<double> m_currents;

  // Parameters
  std::string m_device_file;
};

}  // namespace schunk_svh_driver
