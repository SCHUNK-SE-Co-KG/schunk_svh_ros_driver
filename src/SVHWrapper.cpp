// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Mauch <mauch@fzi.de>
 * \date    2017-02-23
 *
 */
//----------------------------------------------------------------------

#include "DynamicParameterClass.h"
#include "SVHWrapper.h"


#include <std_msgs/MultiArrayDimension.h>

SVHWrapper::SVHWrapper(const ros::NodeHandle& nh)
  : m_priv_nh(nh)
  , m_channels_enabled(false)

{
  bool autostart;
  bool use_internal_logging;

  int reset_timeout;
  std::vector<bool> disable_flags(driver_svh::eSVH_DIMENSION, false);
  // Config that contains the log stream configuration without the file names
  std::string logging_config_file;

  sensor_msgs::JointState joint_msg;

  float max_force;

  m_priv_nh.param<bool>("autostart", autostart, false);
  m_priv_nh.param<bool>("use_internal_logging", use_internal_logging, false);
  m_priv_nh.param<std::string>("serial_device", m_serial_device_name, "/dev/ttyUSB0");
  // Note : Wrong values (like numerics) in the launch file will lead to a "true" value here
  m_priv_nh.getParam("disable_flags", disable_flags);
  m_priv_nh.param<int>("reset_timeout", reset_timeout, 5);
  m_priv_nh.getParam("logging_config", logging_config_file);
  m_priv_nh.param<std::string>("name_prefix", m_name_prefix, "left_hand");
  m_priv_nh.param<int>("connect_retry_count", m_connect_retry_count, 3);
  m_priv_nh.param<float>("maximal_force", max_force, 0.8);
  m_priv_nh.param<int>("use_major_version", m_firmware_major_version, 0);
  m_priv_nh.param<int>("use_minor_version", m_firmware_minor_version, 0);

  // Tell the user what we are using
  ROS_INFO("Name prefix for this Hand was set to: %s", m_name_prefix.c_str());

  if (m_firmware_major_version != 0 || m_firmware_minor_version != 0)
    ROS_INFO("Forced Handversion %d.%d", m_firmware_major_version, m_firmware_minor_version);


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

  // Connect and start the reset so that the hand is ready for use
  // try if SVH is connected
  connect();
  if (autostart)
  {
    if (m_finger_manager->resetChannel(driver_svh::eSVH_ALL))
    {
      ROS_INFO("Driver was autostarted! Input can now be sent. Have a safe and productive day!");
      m_channels_enabled = true;
    }
    else
    {
      ROS_ERROR("Tried to reset the fingers by autostart: Not succeeded!");
    }
  }
  else
  {
    ROS_INFO(
      "SVH Driver Ready, you will need to connect and reset the fingers before you can use "
      "the hand.");
  }

  // set the maximal force / current value from the parameters
  m_finger_manager->setMaxForce(max_force);

  // Subscribe connect topic (Empty)
  connect_sub = m_priv_nh.subscribe("connect", 1, &SVHWrapper::connectCallback, this);

  // Subscribe enable channel topic (Int8)
  enable_sub = m_priv_nh.subscribe("enable_channel", 1, &SVHWrapper::enableChannelCallback, this);

  // services
  m_home_service_all = m_priv_nh.advertiseService("home_reset_offset_all",
     &SVHWrapper::homeAllNodes, this);
  m_home_service_joint_names = m_priv_nh.advertiseService("home_reset_offset_by_id",
     &SVHWrapper::homeNodesChannelIds, this);

  m_setAllForceLimits_srv =
    m_priv_nh.advertiseService("set_all_force_limits", &SVHWrapper::setAllForceLimits, this);
  m_setForceLimitById_srv =
    m_priv_nh.advertiseService("set_force_limit_by_id", &SVHWrapper::setForceLimitById, this);

  m_svh_diagnostics.reset(new SVHDiagnostics(
                                m_priv_nh, m_finger_manager,
                                boost::bind( &SVHWrapper::setRosControlEnable, this, _1),
                                boost::bind( &SVHWrapper::initControllerParameters, this, _1, _2),
                                "diagnostics_to_protocol"));
}

void SVHWrapper::setRosControlEnable(bool enable)
{
  m_channels_enabled = enable;
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
    // a warning so we dont need to check it further. However the log level will only be Info
    // (out of the available Error, Warning, Info, Debug, Trace)
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

void SVHWrapper::initControllerParameters(const uint16_t firmware_major_version,
                                          const uint16_t firmware_minor_version
                                         )
{
  // Parameters that depend on the hardware version of the hand
  XmlRpc::XmlRpcValue dynamic_parameters;

  // read hand parameter (operation data) from parameter server
  std::string parameters_name = "VERSIONS_PARAMETERS";
  try
  {
    if (!m_priv_nh.getParam(parameters_name, dynamic_parameters))
    {
      ROS_FATAL_STREAM("Could not find controller_parameters under " << m_priv_nh.resolveName(parameters_name));
      exit(-1);
    }
  }
  catch (ros::InvalidNameException e)
  {
    ROS_FATAL_STREAM("Illegal parameter name: " << parameters_name);
    exit(-1);
  }

  try
  {
    // Loading hand parameters
    DynamicParameter dyn_parameters(
      firmware_major_version, firmware_minor_version, dynamic_parameters);

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
  m_svh_diagnostics.reset();
}

bool SVHWrapper::connect()
{
  m_channels_enabled = false;

  if (m_finger_manager->isConnected())
  {
    m_finger_manager->disconnect();
  }

  // Receives current Firmware Version
  // because some parameters depend on the version
  if (m_firmware_major_version == 0 && m_firmware_minor_version == 0)
  {
    // reads out current Firmware Version
    driver_svh::SVHFirmwareInfo version_info = m_finger_manager->getFirmwareInfo(m_serial_device_name, m_connect_retry_count);
    ROS_INFO("Current Handversion %d.%d", version_info.version_major, version_info.version_minor);
    m_firmware_major_version = version_info.version_major;
    m_firmware_minor_version = version_info.version_minor;
  }

  // Was firmware info given by the hand?
  if (m_firmware_major_version == 0 && m_firmware_minor_version == 0)
  {
    ROS_ERROR(
      "Could not get Version Info from SCHUNK five finger hand with serial device %s, and retry count %i",
      m_serial_device_name.c_str(),
      m_connect_retry_count);
    return false;
  }

  // read out the operation data with the connected hand version
  initControllerParameters( static_cast<uint16_t>(m_firmware_major_version),
                            static_cast<uint16_t>(m_firmware_minor_version));

  // try connect, channels are not enabled -> have to be resetted/homed
  if (!m_finger_manager->connect(m_serial_device_name, m_connect_retry_count))
  {
    ROS_ERROR(
      "Could not connect to SCHUNK five finger hand with serial device %s, and retry count %i",
      m_serial_device_name.c_str(),
      m_connect_retry_count);
    return false;
  }

  return true;

}

// Callback function for connecting to SCHUNK five finger hand
void SVHWrapper::connectCallback(const std_msgs::Empty&)
{
  ROS_INFO("trying to connect");
  connect();
}

// Callback function to enable channels of SCHUNK five finger hand
void SVHWrapper::enableChannelCallback(const std_msgs::Int8ConstPtr& channel)
{
  m_finger_manager->enableChannel(static_cast<driver_svh::SVHChannel>(channel->data));
}

bool SVHWrapper::homeAllNodes(schunk_svh_driver::HomeAll::Request& req,
                              schunk_svh_driver::HomeAll::Response& resp)
{
  // disable flag to stop ros-control-loop
  m_channels_enabled = false;

  resp.success = m_finger_manager->resetChannel(driver_svh::eSVH_ALL);

  // enable flag to start ros-control-loop if successfully resetted
  if (resp.success == true)
  {
    ROS_INFO("successfully resetted");
    m_channels_enabled = true;
  }

  return resp.success;
}

bool SVHWrapper::homeNodesChannelIds(schunk_svh_driver::HomeWithChannels::Request& req,
                                     schunk_svh_driver::HomeWithChannels::Response& resp)
{
  // is ros-control-loop enabled ?
  bool channels_enabled_before;
  if(m_channels_enabled)
  {
    // disable flag to stop ros-control-loop
    m_channels_enabled = false;
    channels_enabled_before = true;
  }
  {
    // not all channels resetted before, so ros-control-loop won't be enabled after
    channels_enabled_before = false;
    ROS_WARN_STREAM("After resetting asked channel the ros controll loop will not be enabled");
  }


  for (std::vector<uint8_t>::iterator it = req.channel_ids.begin(); it != req.channel_ids.end(); ++it)
  {
    m_finger_manager->resetChannel(static_cast<driver_svh::SVHChannel>(*it));
  }

  if(channels_enabled_before)
  {
    // enable flag to stop ros-control-loop
    m_channels_enabled = true;
  }

  resp.success = true;
  return resp.success;
}


bool SVHWrapper::setAllForceLimits(schunk_svh_driver::SetAllChannelForceLimits::Request &req,
                                   schunk_svh_driver::SetAllChannelForceLimits::Response &res)
{
  for (size_t channel = 0; channel < driver_svh::eSVH_DIMENSION; ++channel)
  {
    res.force_limit[channel] = setChannelForceLimit(channel, req.force_limit[channel]);
  }
  return true;
}

bool SVHWrapper::setForceLimitById(schunk_svh_driver::SetChannelForceLimit::Request &req,
                                   schunk_svh_driver::SetChannelForceLimit::Response &res)
{
  res.force_limit = setChannelForceLimit(req.channel_id, req.force_limit);
  return true;
}


float SVHWrapper::setChannelForceLimit(size_t channel, float force_limit)
{
  return m_finger_manager->setForceLimit(static_cast<driver_svh::SVHChannel>(channel), force_limit);

}
