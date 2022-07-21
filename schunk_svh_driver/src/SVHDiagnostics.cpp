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


SVHDiagnostics::SVHDiagnostics(
  const ros::NodeHandle& nh,
  std::shared_ptr<driver_svh::SVHFingerManager>& finger_manager,
  std::function<void(bool)> enable_ros_contol_loop,
  std::function<void(uint16_t, uint16_t)> init_controller_parameters,
  std::string name)
  : m_priv_nh(nh)
  , m_diagnostics_action_server(m_priv_nh, name, std::bind(&SVHDiagnostics::basicTestCallback, this, std::placeholders::_1), false)
  , m_action_name(name)
  , m_finger_manager(finger_manager)
  , m_enable_ros_contol_loop(enable_ros_contol_loop)
  , m_init_controller_parameters(init_controller_parameters)
{

  //!
  //! \brief m_pub_protocol_variables pubished the diagnostic results to the SVHProtocol Node
  //! to print the Test-Protocol
  //!
  m_pub_protocol_variables = m_priv_nh.advertise<schunk_svh_msgs::SVHDiagnosticsToProtocol>("diagnostic_info_to_protocol", 1);

  m_basic_test_running = false;

  m_diagnostics_action_server.start();
}

SVHDiagnostics::~SVHDiagnostics()
{
  m_finger_manager.reset();
}

void SVHDiagnostics::initTest()
{
  // First: read out the actual hand version again, not manual set possible
  // firmware of the mounted SVH
  driver_svh::SVHFirmwareInfo firmware_info = m_finger_manager->getFirmwareInfo();
  std::stringstream firmware;
  firmware << (int)firmware_info.version_major << "." << (int)firmware_info.version_minor;
  m_msg_protocol_variable.firmware = firmware.str();

  // Second: set the parameter values for the actual hand to the finger manager,
  // manipulatet preferences like max_force have to be set again, after diagnostics test
  m_init_controller_parameters(firmware_info.version_major, firmware_info.version_minor);

  // usefull variables from the driver_svh
  driver_svh::SVHHomeSettings home_settings;
  driver_svh::SVHCurrentSettings current_settings;

  m_action_feedback.fingers.clear();

  // init the indidividual finger params
  for (size_t channel = 0; channel < driver_svh::SVH_DIMENSION; ++channel)
  {
    schunk_svh_msgs::SVHDiagnosticsFinger finger;
    finger.channel               = channel;
    finger.current_max_actual    = 10;
    finger.current_min_actual    = 100;
    finger.position_range_actual = 0;
    finger.encoder               = false;
    finger.motor                 = false;

    m_finger_manager->getCurrentSettings(
        static_cast<driver_svh::SVHChannel>(channel), current_settings);
    m_finger_manager->getHomeSettings(
        static_cast<driver_svh::SVHChannel>(channel), home_settings);

    finger.current_max_target =
        home_settings.reset_current_factor * current_settings.wmx;
    finger.current_min_target =
        home_settings.reset_current_factor * current_settings.wmn;
    finger.position_range_target =
        abs(abs(home_settings.maximum_offset) -
        abs(home_settings.minimum_offset));
    finger.name = driver_svh::SVHController::m_channel_description[channel];

    m_action_feedback.fingers.push_back(finger);
  }

  // initialize reset success vector
  reset_success.resize(driver_svh::SVH_DIMENSION, false);
}

void SVHDiagnostics::basicTestCallback(const schunk_svh_msgs::SVHDiagnosticsGoalConstPtr & goal)
{
  m_msg_protocol_variable.date_as_string = goal->date_as_string;

  m_enable_ros_contol_loop(false);

  if (m_basic_test_running == false && m_finger_manager->isConnected())
  {
    initTest();

    // reset the actual data of the finger vector
    resetDiagnosticStatus();
    bool basic_test_success = true;

    m_diagnostics_action_server.publishFeedback(m_action_feedback);

    m_basic_test_running = true;
    m_serial_no          = goal->serial_no;

    m_assembly_of = goal->assembly_of;

    m_execution_L = goal->execution_L;
    m_execution_R = goal->execution_R;
    m_communication = goal->communication;

    // get current / home settings from the finger manager
    driver_svh::SVHHomeSettings home_settings;
    struct driver_svh::SVHFingerManager::DiagnosticState diagnostic_data;

    // clear reset success vector to false
    ROS_INFO("SVHDiagnostics - Resetting all fingers ...");
    reset_success.resize(driver_svh::SVH_DIMENSION, false);

    // reset the finger vector with the diagnostic data, if basic test get executed multiple times
    m_finger_manager->resetDiagnosticData(static_cast<driver_svh::SVHChannel>(driver_svh::SVH_ALL));

    for (size_t channel = 0; channel < driver_svh::SVH_DIMENSION; ++channel)
    {
      schunk_svh_msgs::SVHDiagnosticsFinger finger = m_action_feedback.fingers[channel];

      for (int i = 0; i <= 1; i++) // for both hardware constraints (pos/neg)
      {
        // check that preempt has not been requested by the client
        if (!m_diagnostics_action_server.isPreemptRequested() || ros::ok())
        {
          // set the negative direction to reach the negative reset point first, than the normal one
          m_finger_manager->getHomeSettings(static_cast<driver_svh::SVHChannel>(channel), home_settings);
          home_settings.direction = home_settings.direction * (-1);
          m_finger_manager->setHomeSettings(static_cast<driver_svh::SVHChannel>(channel), home_settings);

          // The Heart of the program
          reset_success[channel] = m_finger_manager->resetChannel(static_cast<driver_svh::SVHChannel>(channel));
          ROS_DEBUG_STREAM("Channel " << channel << " reset success = " << reset_success[channel]
                             ? "true"
                             : "false");

          m_finger_manager->getDiagnosticStatus(static_cast<driver_svh::SVHChannel>(channel), diagnostic_data);

          finger.current_max_actual = diagnostic_data.diagnostic_current_maximum;
          finger.current_min_actual = diagnostic_data.diagnostic_current_minimum;
          finger.encoder            = diagnostic_data.diagnostic_encoder_state;
          finger.motor              = diagnostic_data.diagnostic_motor_state;
          finger.position_range_actual =
            diagnostic_data.diagnostic_position_maximum - diagnostic_data.diagnostic_position_minimum;

          m_action_feedback.fingers[channel] = finger;

          m_diagnostics_action_server.publishFeedback(m_action_feedback);
        }
        else
        {
          ROS_INFO("%s: Preempted", m_action_name.c_str());
          // set the action state to preempted
          m_diagnostics_action_server.setPreempted();
          break;
        }
      }
      // summery of all resets
      basic_test_success &= reset_success[channel];
    }

    // debugOuput();

    m_basic_test_running = false;

    if (basic_test_success)
    { // if successed than just send this
      ROS_INFO_STREAM("SVHDiagnostics - Basic test by reset routine for the Hand: SVH "
                      << m_serial_no
                      << " was: " << (basic_test_success ? "successfull" : "FAILED!"));
    }

    m_diagnostics_action_server.setSucceeded(evaluateBasicTest());

    qualityProtocolWritting();
  }
  else
  {
    schunk_svh_msgs::SVHDiagnosticsResult action_result;

    action_result.result = -1;
    action_result.channel = -1;

    m_diagnostics_action_server.setAborted(action_result);

    ROS_ERROR("No hand connected!");
  }

  m_enable_ros_contol_loop(true);

}

schunk_svh_msgs::SVHDiagnosticsResult SVHDiagnostics::evaluateBasicTest()
{
  ROS_INFO_STREAM("SVHDiagnostics - Evaluate the Basic Test");

  schunk_svh_msgs::SVHDiagnosticsResult action_result;

  action_result.result = zero_defect;
  action_result.channel = -1;

  // failure of :
  // 1st controller board both encoder and motor
  if (m_action_feedback.fingers[0].encoder == false && m_action_feedback.fingers[1].encoder == false &&
      m_action_feedback.fingers[2].encoder == false && m_action_feedback.fingers[3].encoder == false &&
      m_action_feedback.fingers[4].encoder == false && m_action_feedback.fingers[0].motor == false &&
      m_action_feedback.fingers[1].motor == false && m_action_feedback.fingers[2].motor == false &&
      m_action_feedback.fingers[3].motor == false && m_action_feedback.fingers[4].motor == false)
  {
    action_result.result = board_one;
    ROS_WARN_STREAM("SVHDiagnostics::evaluateBasicTest: failure of : 1nd controller board both "
                    "encoder and motor");
    return action_result;
  }

  // 2nd controller board both encoder and motor
  if (m_action_feedback.fingers[5].encoder == false && m_action_feedback.fingers[6].encoder == false &&
      m_action_feedback.fingers[7].encoder == false && m_action_feedback.fingers[8].encoder == false &&
      m_action_feedback.fingers[5].motor == false && m_action_feedback.fingers[6].motor == false &&
      m_action_feedback.fingers[7].motor == false && m_action_feedback.fingers[8].motor == false)
  {
    action_result.result = board_two;
    ROS_WARN_STREAM("SVHDiagnostics::evaluateBasicTest: failure of : 2nd controller board both "
                    "encoder and motor");
    return action_result;
  }

  // 1st controller board encoder
  if (m_action_feedback.fingers[0].encoder == false && m_action_feedback.fingers[1].encoder == false &&
      m_action_feedback.fingers[2].encoder == false && m_action_feedback.fingers[3].encoder == false &&
      m_action_feedback.fingers[4].encoder == false)
  {
    action_result.result = board_one_encoder;
    ROS_WARN_STREAM("SVHDiagnostics::evaluateBasicTest: failure of : 1nd controller board encoder");
    return action_result;
  }

  // 1st controller board motor
  if (m_action_feedback.fingers[0].motor == false && m_action_feedback.fingers[1].motor == false &&
      m_action_feedback.fingers[2].motor == false && m_action_feedback.fingers[3].motor == false &&
      m_action_feedback.fingers[4].motor == false)
  {
    action_result.result = board_one_motor;
    ROS_WARN_STREAM("SVHDiagnostics::evaluateBasicTest: failure of : 1nd controller board motor");
    return action_result;
  }

  // 2nd controller board encoder
  if (m_action_feedback.fingers[5].encoder == false && m_action_feedback.fingers[6].encoder == false &&
      m_action_feedback.fingers[7].encoder == false && m_action_feedback.fingers[8].encoder == false)
  {
    action_result.result = board_two_encoder;
    ROS_WARN_STREAM("SVHDiagnostics::evaluateBasicTest: failure of : 2nd controller board encoder");
    return action_result;
  }

  // 2nd controller board motor
  if (m_action_feedback.fingers[5].motor == false && m_action_feedback.fingers[6].motor == false &&
      m_action_feedback.fingers[7].motor == false && m_action_feedback.fingers[8].motor == false)
  {
    action_result.result = board_two_motor;
    ROS_WARN_STREAM("SVHDiagnostics::evaluateBasicTest: failure of : 2nd controller board motor");
    return action_result;
  }

  // encoder and motor failure, any finger
  for (size_t i = 0; i < driver_svh::SVH_DIMENSION; i++)
  {
    if (m_action_feedback.fingers[i].motor == false && m_action_feedback.fingers[i].encoder == false)
    {
      action_result.result  = encoder_motor;
      action_result.channel = i;
      ROS_WARN_STREAM(
        "SVHDiagnostics::evaluateBasicTest: encoder and motor failure, finger: " << i);
      return action_result;
    }
  }

  // encoder not working, any finger
  for (size_t i = 0; i < driver_svh::SVH_DIMENSION; i++)
  {
    if (m_action_feedback.fingers[i].encoder == false)
    {
      action_result.result  = encoder;
      action_result.channel = i;
      ROS_WARN_STREAM("SVHDiagnostics::evaluateBasicTest: encoder not working, finger: " << i);
      return action_result;
    }
  }

  // motor not working, any finger
  for (size_t i = 0; i < driver_svh::SVH_DIMENSION; i++)
  {
    if (m_action_feedback.fingers[i].motor == false)
    {
      action_result.result  = motor;
      action_result.channel = i;
      ROS_WARN_STREAM("SVHDiagnostics::evaluateBasicTest: motor not working, finger: " << i);
      return action_result;
    }
  }

  // motor don't get enough current, any finger
  for (size_t i = 0; i < driver_svh::SVH_DIMENSION; i++)
  {
    if (m_action_feedback.fingers[i].current_max_actual <
          m_action_feedback.fingers[i].current_max_target ||
        m_action_feedback.fingers[i].current_min_actual >
          m_action_feedback.fingers[i].current_min_target)
    {
      action_result.result  = current_range;
      action_result.channel = i;
      ROS_WARN_STREAM(
        "SVHDiagnostics::evaluateBasicTest: motor don't get enough current, finger: " << i);
      return action_result;
    }
  }

  // position range is not enough, any finger
  for (size_t i = 0; i < driver_svh::SVH_DIMENSION; i++)
  {
    if (m_action_feedback.fingers[i].position_range_actual <
        m_action_feedback.fingers[i].position_range_target)
    {
      action_result.result  = position_range;
      action_result.channel = i;
      ROS_WARN_STREAM(
        "SVHDiagnostics::evaluateBasicTest: position range is not enough, finger: " << i);
      return action_result;
    }
  }

  // basic test successfully without an error
  action_result.result = zero_defect;
  ROS_DEBUG("No error detected! Basic test successfully");
  return action_result;
}

void SVHDiagnostics::resetDiagnosticStatus()
{
  for (size_t channel = 0; channel < driver_svh::SVH_DIMENSION; ++channel)
  {
    schunk_svh_msgs::SVHDiagnosticsFinger finger = m_action_feedback.fingers[channel];
    finger.current_max_actual                          = 0;
    finger.current_min_actual                          = 0;
    finger.position_range_actual                       = 0;
    finger.encoder                                     = false;
    finger.motor                                       = false;

    m_action_feedback.fingers[channel] = finger;
  }
}

void SVHDiagnostics::debugOuput()
{
  // get current / home settings from the finger manager
  driver_svh::SVHHomeSettings home_settings;
  driver_svh::SVHCurrentSettings current_settings;
  struct driver_svh::SVHFingerManager::DiagnosticState diagnostic_data;

  // ---------------- evaluation output of the finger resets
  ROS_INFO_STREAM("SVHDiagnostics - Results of basic test:");

  for (size_t channel = 0; channel < reset_success.size(); ++channel)
  {
    m_finger_manager->getCurrentSettings(static_cast<driver_svh::SVHChannel>(channel), current_settings);
    m_finger_manager->getHomeSettings(static_cast<driver_svh::SVHChannel>(channel), home_settings);
    m_finger_manager->getDiagnosticStatus(static_cast<driver_svh::SVHChannel>(channel), diagnostic_data);

    double desired_current_neg, desired_current_pos;
    double max_current, min_current;
    double max_position, min_position;
    double deadlock;

    desired_current_pos = home_settings.reset_current_factor * current_settings.wmx;
    desired_current_neg = home_settings.reset_current_factor * current_settings.wmn;
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
  SVHDiagnostics::initializeProtocolMessage();

  // serial no. and testers name got from web side
  m_msg_protocol_variable.serial_number = m_serial_no;
  m_msg_protocol_variable.assembly_of   = m_assembly_of;
  m_msg_protocol_variable.execution_R   = m_execution_R;
  m_msg_protocol_variable.execution_L   = m_execution_L;
  m_msg_protocol_variable.communication = m_communication;

  m_msg_protocol_variable.ppnr = " ";

  for (size_t channel = 0; channel < driver_svh::SVH_DIMENSION; ++channel)
  {
    m_msg_protocol_variable.joint[channel] =
      (m_action_feedback.fingers[channel].encoder && m_action_feedback.fingers[channel].motor &&
       (m_action_feedback.fingers[channel].position_range_actual >
        m_action_feedback.fingers[channel].position_range_target) &&
       (m_action_feedback.fingers[channel].current_max_actual >=
        m_action_feedback.fingers[channel].current_max_target) &&
       (m_action_feedback.fingers[channel].current_min_actual <=
        m_action_feedback.fingers[channel].current_min_target));
  }

  m_pub_protocol_variables.publish(m_msg_protocol_variable);
}

void SVHDiagnostics::initializeProtocolMessage()
{
  // date will be evaluate in SVHProtocol
  m_msg_protocol_variable.joint.resize(driver_svh::SVH_DIMENSION, false);
  m_msg_protocol_variable.serial_number = "";
  m_msg_protocol_variable.firmware      = "";
  m_msg_protocol_variable.assembly_of   = "";
  m_msg_protocol_variable.execution_L   = false;
  m_msg_protocol_variable.execution_R   = false;
  m_msg_protocol_variable.communication = false;

  // not used till now!!
  m_msg_protocol_variable.ppnr       = "XXXX";
  m_msg_protocol_variable.assignment = "";
  m_msg_protocol_variable.repair     = false;

  m_msg_protocol_variable.usb_isolator        = false;
  m_msg_protocol_variable.transport_position  = false;
  m_msg_protocol_variable.usb_cabel           = false;
  m_msg_protocol_variable.power_source        = false;
  m_msg_protocol_variable.cd                  = false;
  m_msg_protocol_variable.description         = false;
  m_msg_protocol_variable.mecovis_description = false;
  m_msg_protocol_variable.short_description   = false;
}
