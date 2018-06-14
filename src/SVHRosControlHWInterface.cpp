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

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(SVHRosControlHWInterface, hardware_interface::RobotHW)

using namespace hardware_interface;

SVHRosControlHWInterface::SVHRosControlHWInterface ()
{
}

SVHRosControlHWInterface::~SVHRosControlHWInterface()
{
}


bool SVHRosControlHWInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
  m_svh.reset(new SVHWrapper(robot_hw_nh));

  m_joint_position_commands.resize(driver_svh::eSVH_DIMENSION);
  m_joint_position_commands_last.resize(driver_svh::eSVH_DIMENSION);
  m_joint_positions.resize(driver_svh::eSVH_DIMENSION);
  m_joint_velocity.resize(driver_svh::eSVH_DIMENSION);
  m_joint_effort.resize(driver_svh::eSVH_DIMENSION);
  m_channel_names.resize(driver_svh::eSVH_DIMENSION);

  // Initialize controller
  for (std::size_t channel = 0; channel < driver_svh::eSVH_DIMENSION; ++channel)
  {
    m_channel_names[channel] =
      m_svh->getNamePrefix() + "_" + driver_svh::SVHController::m_channel_description[channel];

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

  return true;
}

void SVHRosControlHWInterface::read(const ros::Time& time, const ros::Duration& period)
{
  m_joint_positions.resize(driver_svh::eSVH_DIMENSION);
  m_joint_effort.resize(driver_svh::eSVH_DIMENSION);

  if (m_svh->getFingerManager()->isConnected() && m_svh->channelsEnabled())
  {
    // Get positions in rad
    for (size_t channel = 0; channel < driver_svh::eSVH_DIMENSION; ++channel)
    {
      double cur_pos = 0.0;
      double cur_cur = 0.0;
      if (m_svh->getFingerManager()->isHomed(static_cast<driver_svh::SVHChannel>(channel)))
      {
        m_svh->getFingerManager()->getPosition(static_cast<driver_svh::SVHChannel>(channel), cur_pos);
        m_svh->getFingerManager()->getCurrent(static_cast<driver_svh::SVHChannel>(channel), cur_cur);
      }
      else
      {
        ROS_ERROR_STREAM("Node " << driver_svh::SVHController::m_channel_description[channel]
                                  << " is in FAULT state");
      }
      m_joint_positions[channel] = cur_pos;
      m_joint_effort[channel] = m_svh->getFingerManager()->convertmAtoN(static_cast<driver_svh::SVHChannel>(channel), cur_cur);
    }
  }
}

void SVHRosControlHWInterface::write(const ros::Time& time, const ros::Duration& period)
{
  if (!m_svh->channelsEnabled())
  {
    return;
  }

  if (driver_svh::eSVH_DIMENSION == m_joint_position_commands.size())
  {
    if (!m_svh->getFingerManager()->setAllTargetPositions(m_joint_position_commands))
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

// sensor_msgs::JointState SVHRosControlHWInterface::getJointMessage()
// {
//   sensor_msgs::JointState joint_msg;
//   joint_msg.name         = m_channel_names;
//   joint_msg.header.stamp = ros::Time::now();
//   joint_msg.position     = m_joint_positions;
//   joint_msg.velocity     = m_joint_velocity;
//   joint_msg.effort       = m_joint_effort;
//
//   return joint_msg;
// }
