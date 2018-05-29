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

#ifndef SCHUNK_SVH_HARDWARE_INTERFACE_H_
#define SCHUNK_SVH_HARDWARE_INTERFACE_H_

// Driver Specific things
#include "SVHWrapper.h"

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

/*!
 * \brief This class defines a ros-control hardware interface.
 *
 */
class SVHRosControlHWInterface : public hardware_interface::RobotHW
{
public:
  SVHRosControlHWInterface ();
  ~SVHRosControlHWInterface ();

//   SVHRosControlHWInterface (ros::NodeHandle& nh, boost::shared_ptr<driver_svh::SVHFingerManager> &finger_manager, std::string &name_prefix);

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
  boost::shared_ptr<SVHWrapper> m_svh;

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
  std::vector<double> m_joint_position_commands_last;

  bool m_is_fault;

  joint_limits_interface::JointLimits m_joint_limits;
  joint_limits_interface::SoftJointLimits
    m_joint_soft_limits; // only available through URDF, currently not used.
};


#endif /*SCHUNK_SVH_HARDWARE_INTERFACE_H_*/
