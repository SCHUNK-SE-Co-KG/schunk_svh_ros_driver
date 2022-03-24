////////////////////////////////////////////////////////////////////////////////
// Copyright 2022 FZI Research Center for Information Technology
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
