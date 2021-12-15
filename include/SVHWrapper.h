// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

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
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>

#include <schunk_svh_driver/HomeAll.h>
#include <schunk_svh_driver/HomeWithChannels.h>
#include <schunk_svh_driver/SetAllChannelForceLimits.h>
#include <schunk_svh_driver/SetChannelForceLimit.h>

// Driver Specific things
#include <schunk_svh_library/control/SVHFingerManager.h>

#include <SVHDiagnostics.h>

class SVHWrapper{
public:
  SVHWrapper(const ros::NodeHandle& nh);
  ~SVHWrapper();

  boost::shared_ptr<driver_svh::SVHFingerManager> getFingerManager () const { return m_finger_manager; }

  std::string getNamePrefix () const { return m_name_prefix; }

  bool channelsEnabled() { return m_channels_enabled; };

private:
    /**
     * @brief Initialize the icl_logging framework
     *
     * @param use_internal_logging If set to true all icl_logging will go to \p logging_config_file
     * @param logging_config_file Filename of all logging from icl_logging
     */
  void initLogging(const bool use_internal_logging,
                   const std::string& logging_config_file);

  void initControllerParameters(const uint16_t manual_major_version,
                                const uint16_t manual_minor_version
                               );

  //! load parameters and try connecting
  bool connect();

  //! Callback function for connecting to SCHUNK five finger hand
  void connectCallback(const std_msgs::Empty&);

  //! function to set the ros-control-loop enabling flag from the diagnostics class
  void setRosControlEnable(bool enable);

  //! private node handle
  ros::NodeHandle m_priv_nh;

  //! Handle to the SVH finger manager for library access
  boost::shared_ptr<driver_svh::SVHFingerManager> m_finger_manager;

  //! Handle to the diagnostics test class creating a test protocol with the web gui package
  boost::shared_ptr<SVHDiagnostics> m_svh_diagnostics;

  //! Serial device to use for communication with hardware
  std::string m_serial_device_name;

  //! Callback function to reset/home channels of SCHUNK five finger hand
  void resetChannelCallback(const std_msgs::Int8ConstPtr& channel);

  //! Callback function to enable channels of SCHUNK five finger hand
  void enableChannelCallback(const std_msgs::Int8ConstPtr& channel);

  bool homeAllNodes(schunk_svh_driver::HomeAllRequest& req,
                    schunk_svh_driver::HomeAllResponse& resp);

  bool homeNodesChannelIds(schunk_svh_driver::HomeWithChannelsRequest& req,
                           schunk_svh_driver::HomeWithChannelsResponse& resp);

  bool setAllForceLimits(schunk_svh_driver::SetAllChannelForceLimits::Request &req,
                         schunk_svh_driver::SetAllChannelForceLimits::Response &res);
  bool setForceLimitById(schunk_svh_driver::SetChannelForceLimit::Request &req,
                         schunk_svh_driver::SetChannelForceLimit::Response &res);

  float setChannelForceLimit(size_t channel, float force_limit);


  //! Number of times the connect routine tries to connect in case that we receive at least one package
  int m_connect_retry_count;

  //! Prefix for the driver to identify joint names if the Driver should expext "left_hand_Pinky" than the prefix is left_hand
  std::string m_name_prefix;

  //! firmware version, as read from the firmware or set by config. If version is 0, it hasn't been read yet.
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
