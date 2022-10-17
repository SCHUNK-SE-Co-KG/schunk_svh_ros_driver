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

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Mauch <mauch@fzi.de>
 * \date    2017-2-23
 *
 */
//----------------------------------------------------------------------

#ifndef S5FH_WRAPPER_H_INCLUDED
#define S5FH_WRAPPER_H_INCLUDED

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int8.h>

#include <memory>
#include <schunk_svh_msgs/HomeAll.h>
#include <schunk_svh_msgs/HomeWithChannels.h>
#include <schunk_svh_msgs/SetAllChannelForceLimits.h>
#include <schunk_svh_msgs/SetChannelForceLimit.h>

// Driver Specific things
#include <schunk_svh_library/control/SVHFingerManager.h>

#include <SVHDiagnostics.h>

class SVHWrapper
{
public:
  SVHWrapper(const ros::NodeHandle& nh);
  ~SVHWrapper();

  std::shared_ptr<driver_svh::SVHFingerManager> getFingerManager() const
  {
    return m_finger_manager;
  }

  std::string getNamePrefix() const { return m_name_prefix; }

  bool channelsEnabled() { return m_channels_enabled; };

private:
  void initControllerParameters(const uint16_t manual_major_version,
                                const uint16_t manual_minor_version);

  //! load parameters and try connecting
  bool connect();

  //! Callback function for connecting to SCHUNK five finger hand
  void connectCallback(const std_msgs::Empty&);

  //! function to set the ros-control-loop enabling flag from the diagnostics class
  void setRosControlEnable(bool enable);

  //! private node handle
  ros::NodeHandle m_priv_nh;

  //! Handle to the SVH finger manager for library access
  std::shared_ptr<driver_svh::SVHFingerManager> m_finger_manager;

  //! Handle to the diagnostics test class creating a test protocol with the web gui package
  std::shared_ptr<SVHDiagnostics> m_svh_diagnostics;

  //! Serial device to use for communication with hardware
  std::string m_serial_device_name;

  //! Callback function to reset/home channels of SCHUNK five finger hand
  void resetChannelCallback(const std_msgs::Int8ConstPtr& channel);

  //! Callback function to enable channels of SCHUNK five finger hand
  void enableChannelCallback(const std_msgs::Int8ConstPtr& channel);

  bool homeAllNodes(schunk_svh_msgs::HomeAllRequest& req, schunk_svh_msgs::HomeAllResponse& resp);

  bool homeNodesChannelIds(schunk_svh_msgs::HomeWithChannelsRequest& req,
                           schunk_svh_msgs::HomeWithChannelsResponse& resp);

  bool setAllForceLimits(schunk_svh_msgs::SetAllChannelForceLimits::Request& req,
                         schunk_svh_msgs::SetAllChannelForceLimits::Response& res);
  bool setForceLimitById(schunk_svh_msgs::SetChannelForceLimit::Request& req,
                         schunk_svh_msgs::SetChannelForceLimit::Response& res);

  float setChannelForceLimit(size_t channel, float force_limit);


  //! Number of times the connect routine tries to connect in case that we receive at least one
  //! package
  int m_connect_retry_count;

  //! Prefix for the driver to identify joint names if the Driver should expext "left_hand_Pinky"
  //! than the prefix is left_hand
  std::string m_name_prefix;

  //! firmware version, as read from the firmware or set by config. If version is 0, it hasn't been
  //! read yet.
  int m_firmware_major_version;
  int m_firmware_minor_version;

  //! m_channels_enabled enables the ros-control-loop in the hw interface
  bool m_channels_enabled;

  ros::Subscriber connect_sub;
  ros::Subscriber enable_sub;

  ros::ServiceServer m_home_service_all;
  ros::ServiceServer m_home_service_joint_names;
  ros::ServiceServer m_setAllForceLimits_srv;
  ros::ServiceServer m_setForceLimitById_srv;
};

#endif // #ifdef S5FH_WRAPPER_H_INCLUDED
