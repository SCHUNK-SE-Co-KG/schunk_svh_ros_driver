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

// Driver Specific things
#include <driver_svh/SVHFingerManager.h>
// #include <driver_svh/SVHPositionSettings.h>
// #include <driver_svh/SVHCurrentSettings.h>

class SVHWrapper{
public:
  SVHWrapper(const ros::NodeHandle& nh);
  ~SVHWrapper();

  boost::shared_ptr<driver_svh::SVHFingerManager> getFingerManager () const { return m_finger_manager; }

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
                                const uint16_t manual_minor_version,
                                XmlRpc::XmlRpcValue& dynamic_parameters
                               );

  //! Callback function for connecting to SCHUNK five finger hand
  void connectCallback(const std_msgs::Empty&);

  ros::NodeHandle m_priv_nh;

  //! Handle to the SVH finger manager for library access
  boost::shared_ptr<driver_svh::SVHFingerManager> m_finger_manager;

  //! Serial device to use for communication with hardware
  std::string m_serial_device_name;

  //! Callback function to reset/home channels of SCHUNK five finger hand
  void resetChannelCallback(const std_msgs::Int8ConstPtr& channel);

  //! Callback function to enable channels of SCHUNK five finger hand
  void enableChannelCallback(const std_msgs::Int8ConstPtr& channel);

  //! Callback function for setting channel target positions to SCHUNK five finger hand
//   void jointStateCallback(const sensor_msgs::JointStateConstPtr& input);

  //! Number of times the connect routine tries to connect in case that we receive at least one package
  int m_connect_retry_count;

  //! Prefix for the driver to identify joint names if the Driver should expext "left_hand_Pinky" than the prefix is left_hand
  std::string m_name_prefix;

  //! joint state message template
  sensor_msgs::JointState m_channel_pos;

  //! Current Value message template
  std_msgs::Float64MultiArray m_channel_currents;
};

#endif // #ifdef S5FH_WRAPPER_H_INCLUDED
