// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Andreas Hermann <hermann@fzi.de>
 * \date    2016-02-19
 *
 */
//----------------------------------------------------------------------

#include <urdf/model.h>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include "SVHRosControlHWInterface.h"

// Driver Specific things
#include <driver_svh/SVHCurrentSettings.h>
#include <driver_svh/SVHFingerManager.h>
#include <driver_svh/SVHPositionSettings.h>

using namespace hardware_interface;

SVHRosControlHWInterface::SVHRosControlHWInterface(
  ros::NodeHandle& nh,
  boost::shared_ptr<driver_svh::SVHFingerManager>& finger_manager,
  std::string& name_prefix)
  : m_node_handle(nh)
  , m_finger_manager(finger_manager)
  , m_name_prefix(name_prefix)
{
  init();
}

void SVHRosControlHWInterface::init()
{
  m_joint_position_commands.resize(driver_svh::eSVH_DIMENSION);
  m_joint_position_commands_last.resize(driver_svh::eSVH_DIMENSION);
  m_joint_positions.resize(driver_svh::eSVH_DIMENSION);
  m_joint_velocity.resize(driver_svh::eSVH_DIMENSION);
  m_joint_effort.resize(driver_svh::eSVH_DIMENSION);
  m_nodes_in_fault.resize(driver_svh::eSVH_DIMENSION, false);
  m_channel_names.resize(driver_svh::eSVH_DIMENSION);

  // Initialize controller
  for (std::size_t channel = 0; channel < driver_svh::eSVH_DIMENSION; ++channel)
  {
    m_channel_names[channel] =
      m_name_prefix + "_" + driver_svh::SVHController::m_channel_description[channel];

    ROS_DEBUG_STREAM(
      "Controller Hardware interface: Loading joint with id " << channel << " named "
                                                              << m_channel_names[channel]);
    if (m_channel_names[channel] == "")
    {
      ROS_ERROR_STREAM("Could not find joint name for SVH device "
                       << channel
                       << ". You will not be able to use this device with the controller!");
    }
    else
    {
      // Create joint state interface
      m_joint_state_interface.registerHandle(
        hardware_interface::JointStateHandle(m_channel_names[channel],
                                             &m_joint_positions[channel],
                                             &m_joint_velocity[channel],
                                             &m_joint_effort[channel]));

      // Create position joint interface
      hardware_interface::JointHandle hwi_handle(
        m_joint_state_interface.getHandle(m_channel_names[channel]),
        &m_joint_position_commands[channel]);
      m_position_joint_interface.registerHandle(hwi_handle);
    }
  }


  registerInterface(&m_joint_state_interface);    // From RobotHW base class.
  registerInterface(&m_position_joint_interface); // From RobotHW base class.
}

void SVHRosControlHWInterface::read()
{
  m_joint_positions.resize(driver_svh::eSVH_DIMENSION);
  m_joint_effort.resize(driver_svh::eSVH_DIMENSION);

  if (m_finger_manager->isConnected())
  {
    // Get positions in rad
    for (size_t channel = 0; channel < driver_svh::eSVH_DIMENSION; ++channel)
    {
      double cur_pos = 0.0;
      double cur_cur = 0.0;
      if (m_finger_manager->isHomed(static_cast<driver_svh::SVHChannel>(channel)))
      {
        m_nodes_in_fault[channel] = false;
        m_finger_manager->getPosition(static_cast<driver_svh::SVHChannel>(channel), cur_pos);
        m_finger_manager->getCurrent(static_cast<driver_svh::SVHChannel>(channel), cur_cur);
      }
      else
      {
        if (!m_nodes_in_fault[channel])
        {
          ROS_ERROR_STREAM("Node " << driver_svh::SVHController::m_channel_description[channel]
                                   << " is in FAULT state");
        }
        m_nodes_in_fault[channel] = true;
      }
      m_joint_positions[channel] = cur_pos;
      m_joint_effort[channel] =
        cur_cur * driver_svh::SVHController::channel_effort_constants[channel];
    }
  }
}

void SVHRosControlHWInterface::write(ros::Time time, ros::Duration period)
{
  if (driver_svh::eSVH_DIMENSION == m_joint_position_commands.size())
  {
    if (!m_finger_manager->setAllTargetPositions(m_joint_position_commands))
    {
      ROS_WARN_ONCE("Set target position command rejected!");
    }
  }
  else
  {
    // TODO: Send individual commands instead of all joints at once.
    ROS_ERROR("Number of known joints and number of commanded joints do not match!");
  }
}

bool SVHRosControlHWInterface::prepareSwitch(
  const std::list<hardware_interface::ControllerInfo>& start_list,
  const std::list<hardware_interface::ControllerInfo>& stop_list)
{
  return hardware_interface::RobotHW::prepareSwitch(start_list, stop_list);
}

void SVHRosControlHWInterface::doSwitch(
  const std::list<hardware_interface::ControllerInfo>& start_list,
  const std::list<hardware_interface::ControllerInfo>& stop_list)
{
  hardware_interface::RobotHW::doSwitch(start_list, stop_list);
}

sensor_msgs::JointState SVHRosControlHWInterface::getJointMessage()
{
  sensor_msgs::JointState joint_msg;
  joint_msg.name         = m_channel_names;
  joint_msg.header.stamp = ros::Time::now();
  joint_msg.position     = m_joint_positions;
  joint_msg.velocity     = m_joint_velocity;
  joint_msg.effort       = m_joint_effort;

  return joint_msg;
}
