// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    system_interface.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2022/03/02
 *
 */
//-----------------------------------------------------------------------------

#pragma once

#include <map>
#include <thread>

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"

namespace schunk_svh_driver
{
// Additional hardware interfaces for the SVH:
constexpr char HW_IF_CURRENT[] = "current";

/**
 * @brief A ROS2-control SystemInterface for the Schunk SVH
 *
 * This class provides the hardware abstraction for the Schunk SVH in ROS2.
 * It's instantiated via the usual ROS2-conform lifecylce as a
 * controller_manager coordinated library.  Its goal is to support all relevant
 * hardware_interfaces as exposed by the schunk_svh_library.
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
  // Command buffers for the controllers
  std::vector<double> m_position_commands;

  // State buffers for the controllers
  std::vector<double> m_positions;
  std::vector<double> m_velocities;
  std::vector<double> m_currents;
};

}  // namespace schunk_svh_driver
