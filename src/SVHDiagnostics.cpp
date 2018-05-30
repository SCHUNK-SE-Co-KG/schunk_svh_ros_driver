// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Johannes Mangler
 * \date    2018-05-30
 * \
 *
 */
//----------------------------------------------------------------------
#include "SVHDiagnostics.h"

// Custom includes
#include <icl_core/EnumHelper.h>


ros::Publisher pub_diagnosic_status;
ros::Publisher pub_diagnosic_results;
ros::Publisher pub_basic_test_finished;
ros::Publisher m_publish_latex;
ros::Publisher pub_latex_variables;

SVHDiagnostics::SVHDiagnostics(const ros::NodeHandle& nh)
{

  nh.param<std::string>("serial_device", m_serial_device_name, "/dev/ttyUSB0");
  nh.param<int>("connect_retry_count", m_connect_retry_count, 3);

  // usefull variables from the driver_svh
  driver_svh::SVHHomeSettings home_settings;
  driver_svh::SVHCurrentSettings current_settings;

  // start SVH driver wrapper
  m_svh.reset(new SVHWrapper(nh));

  // init the indidividual finger params
  for (size_t channel = 0; channel < driver_svh::eSVH_DIMENSION; ++channel)
  {
    fzi_manipulation_msgs::SVHDiagnosticsFinger finger;
    finger.channel               = channel;
    finger.current_max_actual    = 10;
    finger.current_min_actual    = 100;
    finger.position_range_actual = 0;
    finger.encoder               = false;
    finger.motor                 = false;

    m_svh->getFingerManager()->getCurrentSettings(
        static_cast<driver_svh::SVHChannel>(channel), current_settings);
    m_svh->getFingerManager()->getHomeSettings(
        static_cast<driver_svh::SVHChannel>(channel), home_settings);

    finger.current_max_target =
        home_settings.resetCurrentFactor * current_settings.wmx;
    finger.current_min_target =
        home_settings.resetCurrentFactor * current_settings.wmn;
    finger.position_range_target =
        abs(abs(home_settings.maximumOffset) -
        abs(home_settings.minimumOffset));
    finger.name = driver_svh::SVHController::m_channel_description[channel];

    m_finger_vector.fingers.push_back(finger);
  }

  // initialize reset success vector
  reset_success.resize(driver_svh::eSVH_DIMENSION, false);

  m_basic_test_running = false;
}

SVHDiagnostics::~SVHDiagnostics()
{
  // Tell the driver to close connections
  m_svh->getFingerManager()->disconnect();
}

void SVHDiagnostics::basicTestCallback(const fzi_manipulation_msgs::SVHDiagnosticsMsgForPdf& msg)
{
  if (m_basic_test_running == false)
  {
    // reset the actual data of the finger vector
    resetDiagnosticStatus();
    bool basic_test_success = true;

    // publish empty finger vector for first time or clear data on page
    pub_diagnosic_status.publish(m_finger_vector);

    m_basic_test_running = true;
    m_serial_no          = msg.serial_no;

    m_assembly_of = msg.assembly_of;

    if (msg.execution_L == 1)
    {
      m_execution_L = true;
    }
    else
    {
      m_execution_L = false;
    }

    if (msg.execution_R == 1)
    {
      m_execution_R = true;
    }
    else
    {
      m_execution_R = false;
    }

    if (msg.communication == 1)
    {
      m_communication = true;
    }
    else
    {
      m_communication = false;
    }

    // connect to the SVH hand
    m_svh->getFingerManager()->connect(m_serial_device_name, m_connect_retry_count);

    // get current / home settings from the finger manager
    driver_svh::SVHHomeSettings home_settings;
    struct driver_svh::SVHFingerManager::diagnostic_state diagnostic_data;

    // clear reset success vector to false
    ROS_INFO("SVHDiagnostics - Resetting all fingers ...");
    reset_success.resize(driver_svh::eSVH_DIMENSION, false);

    // reset the finger vector with the diagnostic data, if basic test get executed multiple times
    m_svh->getFingerManager()->resetDiagnosticData(static_cast<driver_svh::SVHChannel>(driver_svh::eSVH_ALL));

    for (size_t channel = 0; channel < driver_svh::eSVH_DIMENSION; ++channel)
    {
      fzi_manipulation_msgs::SVHDiagnosticsFinger finger = m_finger_vector.fingers[channel];

      for (int i = 0; i <= 1; i++) // for both hardware constraints (pos/neg)
      {
        // set the negative direction to reach the negative reset point first, than the normal one
        m_svh->getFingerManager()->getHomeSettings(static_cast<driver_svh::SVHChannel>(channel), home_settings);
        home_settings.direction = home_settings.direction * (-1);
        m_svh->getFingerManager()->setHomeSettings(static_cast<driver_svh::SVHChannel>(channel), home_settings);

        // The Heart of the program
        reset_success[channel] = m_svh->getFingerManager()->resetChannel(static_cast<driver_svh::SVHChannel>(channel));
        ROS_DEBUG_STREAM("Channel " << channel << " reset success = " << reset_success[channel]
                           ? "true"
                           : "false");

        m_svh->getFingerManager()->getDiagnosticStatus(static_cast<driver_svh::SVHChannel>(channel), diagnostic_data);

        finger.current_max_actual = diagnostic_data.diagnostic_current_maximum;
        finger.current_min_actual = diagnostic_data.diagnostic_current_minimum;
        finger.encoder            = diagnostic_data.diagnostic_encoder_state;
        finger.motor              = diagnostic_data.diagnostic_motor_state;
        finger.position_range_actual =
          diagnostic_data.diagnostic_position_maximum - diagnostic_data.diagnostic_position_minimum;

        m_finger_vector.fingers[channel] = finger;

        pub_diagnosic_status.publish(m_finger_vector);
      }
      // summery of all resets
      basic_test_success &= reset_success[channel];
    }

    // debugOuput();

    m_basic_test_running = false;
    std_msgs::Empty msg_empty;
    pub_basic_test_finished.publish(msg_empty);

    if (basic_test_success)
    { // if successed than just send this
      ROS_INFO_STREAM("SVHDiagnostics - Basic test by reset routine for the Hand: SVH "
                      << m_serial_no
                      << " was: " << (basic_test_success ? "successfull" : "FAILED!"));
    }

    pub_diagnosic_results.publish(evaluateBasicTest());

    qualityProtocolWritting();
  }
  else
  {
    // TODO: publish the error message to the side
    ROS_ERROR("Basic Test still startet!");
  }
}

fzi_manipulation_msgs::SVHDiagnosticsResult SVHDiagnostics::evaluateBasicTest()
{
  ROS_INFO_STREAM("SVHDiagnostics - Evaluate the Basic Test");

  fzi_manipulation_msgs::SVHDiagnosticsResult mesg;

  // failure of :
  // 1st controller board both encoder and motor
  if (m_finger_vector.fingers[0].encoder == false && m_finger_vector.fingers[1].encoder == false &&
      m_finger_vector.fingers[2].encoder == false && m_finger_vector.fingers[3].encoder == false &&
      m_finger_vector.fingers[4].encoder == false && m_finger_vector.fingers[0].motor == false &&
      m_finger_vector.fingers[1].motor == false && m_finger_vector.fingers[2].motor == false &&
      m_finger_vector.fingers[3].motor == false && m_finger_vector.fingers[4].motor == false)
  {
    mesg.result = board_one;
    ROS_INFO_STREAM("SVHDiagnostics::evaluateBasicTest: failure of : 1nd controller board both "
                    "encoder and motor");
    return mesg;
  }

  // 2nd controller board both encoder and motor
  if (m_finger_vector.fingers[5].encoder == false && m_finger_vector.fingers[6].encoder == false &&
      m_finger_vector.fingers[7].encoder == false && m_finger_vector.fingers[8].encoder == false &&
      m_finger_vector.fingers[5].motor == false && m_finger_vector.fingers[6].motor == false &&
      m_finger_vector.fingers[7].motor == false && m_finger_vector.fingers[8].motor == false)
  {
    mesg.result = board_two;
    ROS_INFO_STREAM("SVHDiagnostics::evaluateBasicTest: failure of : 2nd controller board both "
                    "encoder and motor");
    return mesg;
  }

  // 1st controller board encoder
  if (m_finger_vector.fingers[0].encoder == false && m_finger_vector.fingers[1].encoder == false &&
      m_finger_vector.fingers[2].encoder == false && m_finger_vector.fingers[3].encoder == false &&
      m_finger_vector.fingers[4].encoder == false)
  {
    mesg.result = board_one_encoder;
    ROS_INFO_STREAM("SVHDiagnostics::evaluateBasicTest: failure of : 1nd controller board encoder");
    return mesg;
  }

  // 1st controller board motor
  if (m_finger_vector.fingers[0].motor == false && m_finger_vector.fingers[1].motor == false &&
      m_finger_vector.fingers[2].motor == false && m_finger_vector.fingers[3].motor == false &&
      m_finger_vector.fingers[4].motor == false)
  {
    mesg.result = board_one_motor;
    ROS_INFO_STREAM("SVHDiagnostics::evaluateBasicTest: failure of : 1nd controller board motor");
    return mesg;
  }

  // 2nd controller board encoder
  if (m_finger_vector.fingers[5].encoder == false && m_finger_vector.fingers[6].encoder == false &&
      m_finger_vector.fingers[7].encoder == false && m_finger_vector.fingers[8].encoder == false)
  {
    mesg.result = board_two_encoder;
    ROS_INFO_STREAM("SVHDiagnostics::evaluateBasicTest: failure of : 2nd controller board encoder");
    return mesg;
  }

  // 2nd controller board motor
  if (m_finger_vector.fingers[5].motor == false && m_finger_vector.fingers[6].motor == false &&
      m_finger_vector.fingers[7].motor == false && m_finger_vector.fingers[8].motor == false)
  {
    mesg.result = board_two_motor;
    ROS_INFO_STREAM("SVHDiagnostics::evaluateBasicTest: failure of : 2nd controller board motor");
    return mesg;
  }

  // encoder and motor failure, any finger
  for (size_t i = 0; i < driver_svh::eSVH_DIMENSION; i++)
  {
    if (m_finger_vector.fingers[i].motor == false && m_finger_vector.fingers[i].encoder == false)
    {
      mesg.result  = encoder_motor;
      mesg.channel = i;
      ROS_INFO_STREAM(
        "SVHDiagnostics::evaluateBasicTest: encoder and motor failure, finger: " << i);
      return mesg;
    }
  }

  // encoder not working, any finger
  for (size_t i = 0; i < driver_svh::eSVH_DIMENSION; i++)
  {
    if (m_finger_vector.fingers[i].encoder == false)
    {
      mesg.result  = encoder;
      mesg.channel = i;
      ROS_INFO_STREAM("SVHDiagnostics::evaluateBasicTest: encoder not working, finger: " << i);
      return mesg;
    }
  }

  // motor not working, any finger
  for (size_t i = 0; i < driver_svh::eSVH_DIMENSION; i++)
  {
    if (m_finger_vector.fingers[i].motor == false)
    {
      mesg.result  = motor;
      mesg.channel = i;
      ROS_INFO_STREAM("SVHDiagnostics::evaluateBasicTest: motor not working, finger: " << i);
      return mesg;
    }
  }

  // motor don't get enough current, any finger
  for (size_t i = 0; i < driver_svh::eSVH_DIMENSION; i++)
  {
    if (m_finger_vector.fingers[i].current_max_actual <
          m_finger_vector.fingers[i].current_max_target ||
        m_finger_vector.fingers[i].current_min_actual >
          m_finger_vector.fingers[i].current_min_target)
    {
      mesg.result  = current_range;
      mesg.channel = i;
      ROS_INFO_STREAM(
        "SVHDiagnostics::evaluateBasicTest: motor don't get enough current, finger: " << i);
      return mesg;
    }
  }

  // postition range is not enough, any finger
  for (size_t i = 0; i < driver_svh::eSVH_DIMENSION; i++)
  {
    if (m_finger_vector.fingers[i].position_range_actual <
        m_finger_vector.fingers[i].position_range_target)
    {
      mesg.result  = position_range;
      mesg.channel = i;
      ROS_INFO_STREAM(
        "SVHDiagnostics::evaluateBasicTest: postition range is not enough, finger: " << i);
      return mesg;
    }
  }

  // basic test successfully without an error
  mesg.result = zero_defect;
  ROS_DEBUG("No error detected! Basic test successfully");
  return mesg;
}

fzi_manipulation_msgs::SVHDiagnosticsFingerVector SVHDiagnostics::getFingerFeedback()
{
  return m_finger_vector;
}

void SVHDiagnostics::resetDiagnosticStatus()
{
  for (size_t channel = 0; channel < driver_svh::eSVH_DIMENSION; ++channel)
  {
    fzi_manipulation_msgs::SVHDiagnosticsFinger finger = m_finger_vector.fingers[channel];
    finger.current_max_actual                          = 0;
    finger.current_min_actual                          = 0;
    finger.position_range_actual                       = 0;
    finger.encoder                                     = false;
    finger.motor                                       = false;

    m_finger_vector.fingers[channel] = finger;
    ROS_INFO_STREAM("Diagnostic finger vector reset correctly: " << channel);
  }
}

void SVHDiagnostics::debugOuput()
{
  // get current / home settings from the finger manager
  driver_svh::SVHHomeSettings home_settings;
  driver_svh::SVHCurrentSettings current_settings;
  struct driver_svh::SVHFingerManager::diagnostic_state diagnostic_data;

  // ---------------- evaluation output of the finger resets
  ROS_INFO_STREAM("SVHDiagnostics - Results of basic test:");

  for (size_t channel = 0; channel < reset_success.size(); ++channel)
  {
    m_svh->getFingerManager()->getCurrentSettings(static_cast<driver_svh::SVHChannel>(channel), current_settings);
    m_svh->getFingerManager()->getHomeSettings(static_cast<driver_svh::SVHChannel>(channel), home_settings);
    m_svh->getFingerManager()->getDiagnosticStatus(static_cast<driver_svh::SVHChannel>(channel), diagnostic_data);

    double desired_current_neg, desired_current_pos;
    double max_current, min_current;
    double max_position, min_position;
    double deadlock;

    desired_current_pos = home_settings.resetCurrentFactor * current_settings.wmx;
    desired_current_neg = home_settings.resetCurrentFactor * current_settings.wmn;
    max_current         = diagnostic_data.diagnostic_current_maximum;
    min_current         = diagnostic_data.diagnostic_current_minimum;
    max_position        = diagnostic_data.diagnostic_position_maximum;
    min_position        = diagnostic_data.diagnostic_position_minimum;
    deadlock            = diagnostic_data.diagnostic_deadlock;

    ROS_INFO_STREAM("Channel: " << channel);
    ROS_INFO_STREAM("Finger " << driver_svh::SVHController::m_channel_description[channel]
                              << " reset: \t" << (reset_success[channel] ? "OK" : "FAILED"));
    ROS_INFO_STREAM("Maximal position: " << max_position << "\t minimal position: " << min_position
                                         << "\t maximal current: " << max_current << " / "
                                         << desired_current_pos
                                         << "\t minimal current: " << min_current << " / "
                                         << desired_current_neg << " deadlock: " << deadlock);
    ROS_INFO_STREAM("Position range: " << max_position - min_position << " max position: "
                                       << max_position << " min position: " << min_position);
  }
}

void SVHDiagnostics::qualityProtocolWritting()
{
  SVHDiagnostics::initializeLatexMessage();

  // serial no. and testers name got from web side
  m_msg_latex_variable.serial_number = m_serial_no;
  m_msg_latex_variable.assembly_of   = m_assembly_of;
  m_msg_latex_variable.execution_R   = m_execution_R;
  m_msg_latex_variable.execution_L   = m_execution_L;
  m_msg_latex_variable.communication = m_communication;

  // firmware of the mounted SVH
  driver_svh::SVHFirmwareInfo firmware_info = m_svh->getFingerManager()->getFirmwareInfo();
  std::stringstream firmware;
  firmware << (int)firmware_info.version_major << "." << (int)firmware_info.version_minor;
  m_msg_latex_variable.firmware = firmware.str();

  m_msg_latex_variable.ppnr = " ";

  for (size_t channel = 0; channel < driver_svh::eSVH_DIMENSION; ++channel)
  {
    m_msg_latex_variable.joint[channel] =
      (m_finger_vector.fingers[channel].encoder && m_finger_vector.fingers[channel].motor &&
       (m_finger_vector.fingers[channel].position_range_actual >
        m_finger_vector.fingers[channel].position_range_target) &&
       (m_finger_vector.fingers[channel].current_max_actual >=
        m_finger_vector.fingers[channel].current_max_target) &&
       (m_finger_vector.fingers[channel].current_min_actual <=
        m_finger_vector.fingers[channel].current_min_target));
  }

  pub_latex_variables.publish(m_msg_latex_variable);
}

void SVHDiagnostics::initializeLatexMessage()
{
  // date will be evaluate in SVHLatex
  m_msg_latex_variable.joint.resize(driver_svh::eSVH_DIMENSION, false);
  m_msg_latex_variable.serial_number = "";
  m_msg_latex_variable.firmware      = "";
  m_msg_latex_variable.assembly_of   = "";
  m_msg_latex_variable.execution_L   = false;
  m_msg_latex_variable.execution_R   = false;
  m_msg_latex_variable.communication = false;

  // not used till now!!
  m_msg_latex_variable.ppnr       = "XXXX";
  m_msg_latex_variable.assignment = "";
  m_msg_latex_variable.repair     = false;

  m_msg_latex_variable.usb_isolator        = false;
  m_msg_latex_variable.transport_position  = false;
  m_msg_latex_variable.usb_cabel           = false;
  m_msg_latex_variable.power_source        = false;
  m_msg_latex_variable.cd                  = false;
  m_msg_latex_variable.description         = false;
  m_msg_latex_variable.mecovis_description = false;
  m_msg_latex_variable.short_description   = false;
}

/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/

int main(int argc, char** argv)
{
  //==========
  // ROS
  //==========

  // Set up ROS.
  ros::init(argc, argv, "svh_diagnostics");
  // Private NH for general params
  ros::NodeHandle nh("~");


  // Tell ROS how fast to run this node. (100 = 100 Hz = 10 ms)
  ros::Rate rate(50);

  //==========
  // Logic
  //==========
  // Node object holding all the relevant functions
  SVHDiagnostics svh_diagnostic_node(nh);


  //==========
  // Callbacks
  //==========

  //!
  //! \brief sub_basic_test subscribe to the topic basic_test_start and call the basicTestCallback
  //!
  ros::Subscriber sub_basic_test =
    nh.subscribe("basic_test_start", 1, &SVHDiagnostics::basicTestCallback, &svh_diagnostic_node);

  //==========
  // Publisher
  //==========

  //!
  //! \brief pub_basic_test_finished pubished the a flag for the end of the basic test
  //!
  pub_basic_test_finished = nh.advertise<std_msgs::Empty>("basic_test_finished", 1);

  //!
  //! \brief pub_diagnosic_status pubished the diagnostic data to the homepage
  //!
  pub_diagnosic_status =
    nh.advertise<fzi_manipulation_msgs::SVHDiagnosticsFingerVector>("diagnostic_status", 1);

  //!
  //! \brief pub_diagnosic_results pubished the diagnostic results to the homepage
  //!
  pub_diagnosic_results =
    nh.advertise<fzi_manipulation_msgs::SVHDiagnosticsResult>("diagnostic_result", 1);

  //!
  //! \brief pub_latex_variables pubished the diagnostic results to the SVHLatex Note to print the
  //! Test-Protocol
  //!
  pub_latex_variables =
    nh.advertise<fzi_manipulation_msgs::SVHDiagnosticsToLatex>("diagnosics_to_latex", 1);

  //==========
  // Messaging
  //==========

  // Main loop.
  while (nh.ok())
  {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
