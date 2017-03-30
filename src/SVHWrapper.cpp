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

#include "DynamicParameterClass.h"
#include "SVHWrapper.h"


#include <std_msgs/MultiArrayDimension.h>

SVHWrapper::SVHWrapper(const ros::NodeHandle& nh)
  : m_priv_nh(nh)
{
  bool autostart;
  bool use_internal_logging;

  int reset_timeout;
  std::vector<bool> disable_flags(driver_svh::eSVH_DIMENSION, false);
  // Config that contains the log stream configuration without the file names
  std::string logging_config_file;

  sensor_msgs::JointState joint_msg;

  // Parameters that depend on the hardware version of the hand
  XmlRpc::XmlRpcValue dynamic_parameters;

//   uint16_t manual_major_version;
  int manual_major_version_int;
//   uint16_t manual_minor_version;
  int manual_minor_version_int;

  m_priv_nh.param<bool>("autostart", autostart, false);
  m_priv_nh.param<bool>("use_internal_logging", use_internal_logging, false);
  m_priv_nh.param<std::string>("serial_device", m_serial_device_name, "/dev/ttyUSB0");
  // Note : Wrong values (like numerics) in the launch file will lead to a "true" value here
  m_priv_nh.getParam("disable_flags", disable_flags);
  m_priv_nh.param<int>("reset_timeout", reset_timeout, 5);
  m_priv_nh.getParam("logging_config", logging_config_file);
  m_priv_nh.param<std::string>("name_prefix", m_name_prefix, "left_hand");
  m_priv_nh.param<int>("connect_retry_count", m_connect_retry_count, 3);
  nh.param<int>("use_major_version", manual_major_version_int, 0);
//   manual_major_version = static_cast<uint16_t>(manual_major_version_int);
  nh.param<int>("use_minor_version", manual_minor_version_int, 0);
//   manual_minor_version = static_cast<uint16_t>(manual_minor_version_int);

  // Tell the user what we are using
  ROS_INFO("Name prefix for this Hand was set to :%s", m_name_prefix.c_str());

  std::string parameters_name = "VERSIONS_PARAMETERS";
  try
  {
    nh.getParam("VERSIONS_PARAMETERS", dynamic_parameters);
  }
  catch (ros::InvalidNameException e)
  {
    ROS_ERROR_STREAM("Could not find controller_parameters under"
                     << /*TODO: resolve*/ parameters_name);
  }

  initLogging(use_internal_logging, logging_config_file);


  for (size_t i = 0; i < 9; ++i)
  {
    if (disable_flags[i])
    {
      ROS_WARN_STREAM("svh_controller disabling channel nr " << i);
    }
  }

  // Init the actual driver hook (after logging initialize)
  m_finger_manager.reset(new driver_svh::SVHFingerManager(disable_flags, reset_timeout));

  initControllerParameters(static_cast<uint16_t>(manual_major_version_int),
                           static_cast<uint16_t>(manual_minor_version_int),
                           dynamic_parameters
  );

  // prepare the channel position message for later sending
  m_channel_pos.name.resize(driver_svh::eSVH_DIMENSION);
  m_channel_pos.position.resize(driver_svh::eSVH_DIMENSION, 0.0);
  m_channel_pos.effort.resize(driver_svh::eSVH_DIMENSION, 0.0);
  for (size_t channel = 0; channel < driver_svh::eSVH_DIMENSION; ++channel)
  {
    m_channel_pos.name[channel] =
      m_name_prefix + "_" + driver_svh::SVHController::m_channel_description[channel];
  }

  // Prepare the channel current message for later sending
  m_channel_currents.data.clear();
  m_channel_currents.data.resize(driver_svh::eSVH_DIMENSION, 0.0);
  m_channel_currents.layout.data_offset = 0;
  std_msgs::MultiArrayDimension dim;
  dim.label  = "channel currents";
  dim.size   = 9;
  dim.stride = 0;
  m_channel_currents.layout.dim.push_back(dim);

  // Connect and start the reset so that the hand is ready for use
  if (autostart && m_finger_manager->connect(m_serial_device_name, m_connect_retry_count))
  {
    m_finger_manager->resetChannel(driver_svh::eSVH_ALL);
    ROS_INFO("Driver was autostarted! Input can now be sent. Have a safe and productive day!");
  }
  else
  {
    ROS_INFO(
      "SVH Driver Ready, you will need to connect and reset the fingers before you can use "
      "the hand.");
  }

  // Subscribe connect topic (Empty)
  ros::Subscriber connect_sub = m_priv_nh.subscribe("connect", 1, &SVHWrapper::connectCallback, this);
  // Subscribe reset channel topic (Int8)
  ros::Subscriber reset_sub =
    m_priv_nh.subscribe("reset_channel", 1, &SVHWrapper::resetChannelCallback, this);
  // Subscribe enable channel topic (Int8)
  ros::Subscriber enable_sub =
    m_priv_nh.subscribe("enable_channel", 1, &SVHWrapper::enableChannelCallback, this);
}

void SVHWrapper::initLogging(const bool use_internal_logging,
                             const std::string& logging_config_file)
{
  // Initialize the icl_library logging framework ( THIS NEEDS TO BE DONE BEFORE ANY LIB OBJECT IS
  // CREATED)
  if (use_internal_logging)
  {
    // Fake an input to the logging call to tell it where to look for the logging config

    // Strdup to create non const chars as it is required by the initialize function.
    // not really beatiful but it works.
    char* argv[] = {strdup("Logging"), strdup("-c"), strdup(logging_config_file.c_str())};
    int argc     = 3; // number of elements above

    // In case the file is not present (event though the parameter is) the logging will just put out
    // a
    // warning so we dont need to check it further. However the log level will only be Info (out of
    // the available Error, Warning, Info, Debug, Trace)
    // in that case also the log files will be disregarded
    if (icl_core::logging::initialize(argc, argv))
      ROS_INFO("Internal logging was activated, output will be written as configured in "
               "logging.xml (default to ~/.ros/log)");
    else
      ROS_WARN("Internal logging was enabled but config file could not be read. Please make sure "
               "the provided path to the config file is correct.");
  }
  else
  {
    icl_core::logging::initialize();
  }
}

void SVHWrapper::initControllerParameters(const uint16_t manual_major_version,
                                          const uint16_t manual_minor_version,
                                          XmlRpc::XmlRpcValue& dynamic_parameters
                                         )
{
  driver_svh::SVHFirmwareInfo version_info;

  // Receives current Firmware Version
  // because some parameters depend on the version
  if (manual_major_version == 0 && manual_minor_version == 0)
  {
    m_finger_manager->connect(m_serial_device_name, m_connect_retry_count);

    // reads out current Firmware Version
    version_info = m_finger_manager->getFirmwareInfo();
    ROS_INFO("current Handversion %d.%d", version_info.version_major, version_info.version_minor);

    m_finger_manager->disconnect();
  }

  try
  {
    // Loading hand parameters
    DynamicParameter dyn_parameters(
      version_info.version_major, version_info.version_minor, dynamic_parameters);

    for (size_t channel = 0; channel < driver_svh::eSVH_DIMENSION; ++channel)
    {
      // Only update the values in case actually have some. Otherwise the driver will use internal
      // defaults. Overwriting them with zeros would be counter productive
      if (dyn_parameters.getSettings().current_settings_given[channel])
      {
        m_finger_manager->setCurrentSettings(
          static_cast<driver_svh::SVHChannel>(channel),
          driver_svh::SVHCurrentSettings(dyn_parameters.getSettings().current_settings[channel]));
      }
      if (dyn_parameters.getSettings().position_settings_given[channel])
      {
        m_finger_manager->setPositionSettings(
          static_cast<driver_svh::SVHChannel>(channel),
          driver_svh::SVHPositionSettings(
            dyn_parameters.getSettings().position_settings[channel]));
      }
      if (dyn_parameters.getSettings().home_settings_given[channel])
      {
        m_finger_manager->setHomeSettings(
          static_cast<driver_svh::SVHChannel>(channel),
          driver_svh::SVHHomeSettings(dyn_parameters.getSettings().home_settings[channel]));
      }
    }
  }
  catch (ros::InvalidNameException e)
  {
    ROS_ERROR(
      "Parameter Error! While reading the controller settings. Will use default settings");
  }
}


SVHWrapper::~SVHWrapper()
{
  m_finger_manager->disconnect();
}

// TODO: Dynamic reconfigure?

// Callback function for connecting to SCHUNK five finger hand
void SVHWrapper::connectCallback(const std_msgs::Empty&)
{
  if (m_finger_manager->isConnected())
  {
    m_finger_manager->disconnect();
  }

  if (!m_finger_manager->connect(m_serial_device_name, m_connect_retry_count))
  {
    ROS_ERROR(
      "Could not connect to SCHUNK five finger hand with serial device %s, and retry count %i",
      m_serial_device_name.c_str(),
      m_connect_retry_count);
  }
}

// Callback function to reset/home channels of SCHUNK five finger hand
void SVHWrapper::resetChannelCallback(const std_msgs::Int8ConstPtr& channel)
{
  // convert int8 channel into SVHChannel enum
  driver_svh::SVHChannel svh_channel = static_cast<driver_svh::SVHChannel>(channel->data);

  if (m_finger_manager->resetChannel(svh_channel))
  {
    ROS_INFO("Channel %i successfully homed!", svh_channel);
  }
  else
  {
    ROS_ERROR("Could not reset channel %i !", svh_channel);
  }
}

// Callback function to enable channels of SCHUNK five finger hand
void SVHWrapper::enableChannelCallback(const std_msgs::Int8ConstPtr& channel)
{
  m_finger_manager->enableChannel(static_cast<driver_svh::SVHChannel>(channel->data));
}
