// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-


// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Georg Heppner <heppner@fzi.de>
 * \date    2014-8-22
 *
 */
//----------------------------------------------------------------------

#ifndef S5FH_NODE_H
#define S5FH_NODE_H

// Messages
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/MultiArrayLayout.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <schunk_svh_driver/svhConfig.h>

// Driver Specific things
#include <driver_svh/SVHCurrentSettings.h>
#include <driver_svh/SVHFingerManager.h>
#include <driver_svh/SVHPositionSettings.h>

#include <boost/shared_ptr.hpp>

class SVHNode
{
public:
  //!
  //! \brief SVHNode constructs a new node object that handles most of the functionality
  //! \param nh ROS Nodehandle
  //!
  SVHNode(const ros::NodeHandle& nh);
  //! Default DTOR
  ~SVHNode();

  //! Dynamic reconfigure callback to update changing parameters
  void dynamic_reconfigure_callback(svh_controller::svhConfig& config, uint32_t level);

  //! Callback function for connecting to SCHUNK five finger hand
  void connectCallback(const std_msgs::Empty&);

  //! Callback function to reset/home channels of SCHUNK five finger hand
  void resetChannelCallback(const std_msgs::Int8ConstPtr& channel);

  //! Callback function to enable channels of SCHUNK five finger hand
  void enableChannelCallback(const std_msgs::Int8ConstPtr& channel);

  //! Callback function for setting channel target positions to SCHUNK five finger hand
  void jointStateCallback(const sensor_msgs::JointStateConstPtr& input);

  //!
  //! \brief SVHNode::getChannelFeedback Gets the latest received positions and efforts from the
  //! driver
  //! \returns The current joint states (Position and Efforts)
  //!
  sensor_msgs::JointState getChannelFeedback();

  //!
  //! \brief getChannelCurrents Returns the current values of the channels as raw output
  //! \return Array containing the current values of the channels in mA
  //!
  std_msgs::Float64MultiArray getChannelCurrents();


private:
  //! Handle to the SVH finger manager for library access
  boost::shared_ptr<driver_svh::SVHFingerManager> fm_;

  //! Serial device to use for communication with hardware
  std::string serial_device_name_;

  //! Number of times the connect routine tries to connect in case that we receive at least one
  //! package
  int connect_retry_count;

  //! Prefix for the driver to identify joint names if the Driver should expect "left_hand_Pinky"
  //! than the prefix is left_hand
  std::string name_prefix;

  //! joint state message template
  sensor_msgs::JointState channel_pos_;

  //! Current Value message template
  std_msgs::Float64MultiArray channel_currents;
};

#endif // S5FH_CONTROLLER_H
