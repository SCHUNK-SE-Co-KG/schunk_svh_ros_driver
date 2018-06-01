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

#include <stdint.h>
#include <string>

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
#include "SVHWrapper.h"

#include "fzi_manipulation_msgs/SVHDiagnosticsFinger.h"
#include "fzi_manipulation_msgs/SVHDiagnosticsFingerVector.h"
#include "fzi_manipulation_msgs/SVHDiagnosticsResult.h"
#include "fzi_manipulation_msgs/SVHDiagnosticsToLatex.h"
#include "fzi_manipulation_msgs/SVHDiagnosticsMsgForPdf.h"

// Driver Specific things
#include <driver_svh/SVHFingerManager.h>
#include <driver_svh/SVHPositionSettings.h>
#include <driver_svh/SVHCurrentSettings.h>
#include <driver_svh/SVHFirmwareInfo.h>

#include <boost/shared_ptr.hpp>

class SVHDiagnostics{

public:
  //!
  //! \brief SVHDiagnostics constructs a new node object that handles most of the functionality
  //! \param nh ROS Nodehandle
  //!
  SVHDiagnostics(const ros::NodeHandle &nh);
  //! Default DTOR
  ~SVHDiagnostics();

  //! Callback function to conduct the basic test for the SCHUNK five finger hand (msg defined in src/fzi_manipulation_msgs/msg/DiagnosticsMsg.msg)
  void basicTestCallback(const fzi_manipulation_msgs::SVHDiagnosticsMsgForPdf& msg);

  //! Callback function to test the subfunctions of SVHDiagnostics
  void testCallback(const std_msgs::String&);

protected:
  //!
  //! \brief m_svh shared pointer of SVHWrapper
  //!
  boost::shared_ptr<SVHWrapper> m_svh;


private:
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
  schunk_web_gui::SVHDiagnosticsResult evaluateBasicTest();

  //!
  //! \brief getFingerFeedback returns the finger vector with the diagnostic data
  //! \return finger vector with diagnostic data
  //!
  schunk_web_gui::SVHDiagnosticsFingerVector getFingerFeedback();

  //!
  //! \brief set the latex variables
  //!
  void qualityProtocolWritting();

  //!
  //! \brief initialize the latex variables
  //!
  void initializeLatexMessage();

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

  //! Type of the Hand
  bool m_execution_R;
  bool m_execution_L;

  //! Type of onnection
  bool m_communication;

  //! To catch multiple bastic test starts
  bool m_basic_test_running;

  //! Finger messages for the diagnostic tab
  schunk_web_gui::SVHDiagnosticsFingerVector m_finger_vector;

  //! Message for the latex variable
  schunk_web_gui::SVHDiagnosticsToLatex m_msg_latex_variable;


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
