// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-


// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Lars Pfotzer
 * \author  Johannes Mangler
 * \date    2015-10-23
 *
 */
//----------------------------------------------------------------------

#ifndef S5FH_DIAGNOSTICS_H
#define S5FH_DIAGNOSTICS_H

// ROS includes.
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <stdint.h>
#include <string>
#include <memory>
#include <functional>

// Date
#include <ctime>
#include <fstream>
#include <iostream>

// Messages
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>

// FZI includes
#include "schunk_svh_driver/SVHDiagnosticsAction.h"
#include "schunk_svh_driver/SVHDiagnosticsToProtocol.h"

// Driver Specific things
#include <schunk_svh_library/control/SVHFingerManager.h>
#include <schunk_svh_library/control/SVHPositionSettings.h>
#include <schunk_svh_library/control/SVHCurrentSettings.h>
#include <schunk_svh_library/SVHFirmwareInfo.h>

class SVHDiagnostics{

public:
  //!
  //! \brief SVHDiagnostics constructs a new node object that handles most of the functionality
  //! \param nh ROS Nodehandle
  //! \param finger_manager Handle to the SVH finger manager for library access
  //! \param enable_ros_contol_loop function handle to set the ros-control-loop enabling flag
  //! \param init_controller_parameters function handle to get the hand parameters
  //! \param name action server name
  //!
  SVHDiagnostics(
    const ros::NodeHandle &nh,
    std::shared_ptr<driver_svh::SVHFingerManager>& finger_manager,
    std::function<void(bool)> enable_ros_contol_loop,
    std::function<void(uint16_t, uint16_t)> init_controller_parameters,
    std::string name);

  //! Default DTOR
  ~SVHDiagnostics();

  //! Callback function to conduct the basic test for the SCHUNK five finger hand (msg defined in
  //! schunk_web_gui/msg/DiagnosticsMsg.msg)
  void basicTestCallback(const schunk_svh_driver::SVHDiagnosticsGoalConstPtr & goal);

  //! Callback function to test the subfunctions of SVHDiagnostics
  void testCallback(const std_msgs::String&);


private:

  //!
  //! \brief private node handle
  //!
  ros::NodeHandle m_priv_nh;

  //!
  //! \brief resetDiagnosticStatus resets the finger vector with the diagnostic data
  //!
  void resetDiagnosticStatus();

  //!
  //! \brief debugOutput prints the diagnostic data of all finger
  //!
  void debugOuput();

  //!
  //! \brief evaluateBasicTest evaluates the diagnostics status of the basic test to send the hint informations to the webside
  //! \return std_msgs::UInt8MultiArray mesg with the test results
  //!
  schunk_svh_driver::SVHDiagnosticsResult evaluateBasicTest();

  //!
  //! \brief set the protocol variables
  //!
  void qualityProtocolWritting();

  //!
  //! \brief initialize the protocol variables
  //!
  void initializeProtocolMessage();

  //!
  //! \brief initialize the protocol variables
  //!
  void initTest();

  //! Handle to the SVH finger manager for library access
  std::shared_ptr<driver_svh::SVHFingerManager> m_finger_manager;

  //!
  //! function handle to set the ros-control-loop enabling flag in the SVHWrapper
  //!
  std::function<void(bool)> m_enable_ros_contol_loop;

  //!
  //! function handle to get the hand parameters from ros-param-server
  //! and set them to the finger_manager to have cleared parameter set
  //!
  std::function<void(uint16_t, uint16_t)> m_init_controller_parameters;

  //! Serial device to use for communication with hardware
  std::string m_serial_device_name;

  //! Number of times the connect routine tries to connect in case that we receive at least one package
  int m_connect_retry_count;

  //! Prefix for the driver to identify joint names if the Driver should expext "left_hand_Pinky" than the prefix is left_hand
  std::string name_prefix;

  //! joint state message template
  sensor_msgs::JointState channel_pos_;

  //! Current Value message template
  std_msgs::Float64MultiArray channel_currents;

  //! Reset successed vector
  std::vector<bool> reset_success;

  //! Serial No of the tested Hand
  std::string m_serial_no;

  //! Name of the testing person
  std::string m_assembly_of;

  //! Type of the Hand, No/ L / R
  bool m_execution_R;
  bool m_execution_L;

  //! Type of onnection
  bool m_communication;

  //! To catch multiple bastic test starts
  bool m_basic_test_running;

  //! publisher to the protocol variables
  ros::Publisher m_pub_protocol_variables;

  //! Message for the Protocol variable
  schunk_svh_driver::SVHDiagnosticsToProtocol m_msg_protocol_variable;

  //! Action Server
  actionlib::SimpleActionServer<schunk_svh_driver::SVHDiagnosticsAction> m_diagnostics_action_server;

  // Action Feedback
  schunk_svh_driver::SVHDiagnosticsFeedback m_action_feedback;

  //! Action result
  schunk_svh_driver::SVHDiagnosticsResult m_action_result;

  // Action server name
  std::string m_action_name;

  enum diag_information{
    zero_defect,
    board_one,
    board_two,
    board_one_encoder,
    board_one_motor,
    board_two_encoder,
    board_two_motor,
    encoder_motor,
    encoder,
    motor,
    current_range,
    position_range,
    diag_information_dimension
  };

};

#endif // S5FH_CONTROLLER_H
