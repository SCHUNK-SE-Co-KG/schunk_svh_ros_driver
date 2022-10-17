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
// Foobar. If not, see <https://www.gnu.org/licenses/>.
//
////////////////////////////////////////////////////////////////////////////////

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Andreas Hermann <hermann@fzi.de>
 * \date    2016-02-19
 *
 */
//----------------------------------------------------------------------

#ifndef SCHUNK_SVH_HARDWARE_INTERFACE_H_
#define SCHUNK_SVH_HARDWARE_INTERFACE_H_

// Driver Specific things
#include "SVHWrapper.h"

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

/*!
 * \brief This class defines a ros-control hardware interface.
 *
 */
class SVHRosControlHWInterface : public hardware_interface::RobotHW
{
public:
  SVHRosControlHWInterface();
  ~SVHRosControlHWInterface();

  /// \brief Initialize the hardware interface
  virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh);
  //   virtual void init();

  /// \brief Read the state from the robot hardware.
  virtual void read(const ros::Time& time, const ros::Duration& period);

  /// \brief write the command to the robot hardware.
  virtual void write(const ros::Time& time, const ros::Duration& period);

  bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                     const std::list<hardware_interface::ControllerInfo>& stop_list);
  void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                const std::list<hardware_interface::ControllerInfo>& stop_list);


  bool isEnabled() const;
  /*!
   * \brief Returns true, when at least one node in the hardware is in a fault state.
   */
  bool isFault() { return m_is_fault; }
  /*!
   * \brief Creates a joint_state message from the current joint angles and returns it.
   */
  //   sensor_msgs::JointState getJointMessage();

protected:
  ros::NodeHandle m_node_handle;
  //! Handle to the SVH finger manager for library access
  std::shared_ptr<SVHWrapper> m_svh;

  // Interfaces
  hardware_interface::JointStateInterface m_joint_state_interface;
  hardware_interface::PositionJointInterface m_position_joint_interface;

  size_t m_num_joints;

  std::vector<uint8_t> m_node_ids;
  std::vector<std::string> m_channel_names; // Combines prefix with channel_description
  std::vector<double> m_joint_positions;
  std::vector<double> m_joint_velocity;
  std::vector<double> m_joint_effort;
  std::vector<double> m_joint_position_commands;

  bool m_is_fault;

  joint_limits_interface::JointLimits m_joint_limits;
  joint_limits_interface::SoftJointLimits
    m_joint_soft_limits; // only available through URDF, currently not used.

private:
  bool m_hardware_ready;
};


#endif /*SCHUNK_SVH_HARDWARE_INTERFACE_H_*/
