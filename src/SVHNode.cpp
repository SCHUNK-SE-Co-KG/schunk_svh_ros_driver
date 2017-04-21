// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Lars Pfotzer
 * \author  Georg Heppner
 * \date    2014-09-23
 *
 */
//----------------------------------------------------------------------
// ROS includes.
#include <ros/ros.h>
#include <string>

// Custom includes
#include "DynamicParameterClass.h"
#include "SVHNode.h"
#include <icl_core/EnumHelper.h>


/*--------------------------------------------------------------------
 * Callback functions
 *------------------------------------------------------------------*/

SVHNode::SVHNode(const ros::NodeHandle& nh)
{
  //==========
  // Params
  //==========

  bool autostart, use_internal_logging;
  int reset_timeout;
  std::vector<bool> disable_flags(driver_svh::eSVH_DIMENSION, false);
  // Config that contains the log stream configuration without the file names
  std::string logging_config_file;

  // Parameters that depend on the hardware version of the hand.
  XmlRpc::XmlRpcValue dynamic_parameters;

  uint16_t manual_major_version = 0;
  int manual_major_version_int = 0;
  uint16_t manual_minor_version = 0;
  int manual_minor_version_int = 0;

  try
  {
    nh.param<bool>("autostart", autostart, false);
    nh.param<bool>("use_internal_logging", use_internal_logging, false);
    nh.param<std::string>("serial_device", serial_device_name_, "/dev/ttyUSB0");
    // Note : Wrong values (like numerics) in the launch file will lead to a "true" value here
    nh.getParam("disable_flags", disable_flags);
    nh.param<int>("reset_timeout", reset_timeout, 5);
    nh.getParam("logging_config", logging_config_file);
    nh.param<std::string>("name_prefix", name_prefix, "left_hand");
    nh.param<int>("connect_retry_count", connect_retry_count, 3);
    nh.getParam("VERSIONS_PARAMETERS", dynamic_parameters); //TODO Check whether parameters exist
    nh.param<int>("use_major_version", manual_major_version_int, 0);
    manual_major_version = static_cast<uint16_t>(manual_major_version_int);
    nh.param<int>("use_minor_version", manual_minor_version_int, 0);
    manual_minor_version = static_cast<uint16_t>(manual_minor_version_int);
  }
  catch (ros::InvalidNameException e)
  {
    ROS_ERROR("Parameter Error!");
  }

  // Tell the user what we are using
  ROS_INFO("Name prefix for this Hand was set to :%s", name_prefix.c_str());

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

  for (size_t i = 0; i < 9; ++i)
  {
    if (disable_flags[i])
    {
      ROS_WARN_STREAM("svh_controller disabling channel nr " << i);
    }
  }

  // Init the actual driver hook (after logging initialize)
  fm_.reset(new driver_svh::SVHFingerManager(disable_flags, reset_timeout));

  // Receives current Firmware Version
  // because some parameters depend on the version
  if (manual_major_version == 0 && manual_minor_version == 0)
  {
    fm_->connect(serial_device_name_, connect_retry_count);
    driver_svh::SVHFirmwareInfo version_info = fm_->getFirmwareInfo();
    ROS_INFO("Hand hardware controller version:  %d.%d",
             version_info.version_major,
             version_info.version_minor);

    manual_major_version = version_info.version_major;
    manual_minor_version = version_info.version_minor;

    // TODO: Is this correct to do here?
    fm_->disconnect();
  }
  // get the the individual finger parameters
  // We will read out all of them, so that in case we fail half way we do not set anything
  try
  {
    // Loading hand parameters
    DynamicParameter dyn_parameters(manual_major_version, manual_minor_version, dynamic_parameters);


    for (size_t channel = 0; channel < driver_svh::eSVH_DIMENSION; ++channel)
    {
      // Only update the values in case actually have some. Otherwise the driver will use internal
      // defaults. Overwriting them with zeros would be counter productive
      if (dyn_parameters.getSettings().current_settings_given[channel])
      {
        fm_->setCurrentSettings(
          static_cast<driver_svh::SVHChannel>(channel),
          driver_svh::SVHCurrentSettings(dyn_parameters.getSettings().current_settings[channel]));
      }
      if (dyn_parameters.getSettings().position_settings_given[channel])
      {
        fm_->setPositionSettings(
          static_cast<driver_svh::SVHChannel>(channel),
          driver_svh::SVHPositionSettings(dyn_parameters.getSettings().position_settings[channel]));
      }
      if (dyn_parameters.getSettings().home_settings_given[channel])
      {
        fm_->setHomeSettings(static_cast<driver_svh::SVHChannel>(channel),
                             driver_svh::SVHHomeSettings(dyn_parameters.getSettings().home_settings[channel]));
      }
    }
  }
  catch (ros::InvalidNameException e)
  {
    ROS_ERROR("Parameter Error! While reading the controller settings. Will use default settings");
  }

  // prepare the channel position message for later sending
  channel_pos_.name.resize(driver_svh::eSVH_DIMENSION);
  channel_pos_.position.resize(driver_svh::eSVH_DIMENSION, 0.0);
  channel_pos_.effort.resize(driver_svh::eSVH_DIMENSION, 0.0);
  for (size_t channel = 0; channel < driver_svh::eSVH_DIMENSION; ++channel)
  {
    channel_pos_.name[channel] =
      name_prefix + "_" + driver_svh::SVHController::m_channel_description[channel];
  }

  // Prepare the channel current message for later sending
  channel_currents.data.clear();
  channel_currents.data.resize(driver_svh::eSVH_DIMENSION, 0.0);
  channel_currents.layout.data_offset = 0;
  std_msgs::MultiArrayDimension dim;
  dim.label  = "channel currents";
  dim.size   = 9;
  dim.stride = 0;
  channel_currents.layout.dim.push_back(dim);

  // Connect and start the reset so that the hand is ready for use
  if (autostart && fm_->connect(serial_device_name_, connect_retry_count))
  {
    fm_->resetChannel(driver_svh::eSVH_ALL);
    ROS_INFO("Driver was autostarted! Input can now be sent. Have a safe and productive day!");
  }
  else
  {
    ROS_INFO("SVH Driver Ready, you will need to connect and reset the fingers before you can use "
             "the hand.");
  }
}

SVHNode::~SVHNode()
{
  // Tell the driver to close connections
  fm_->disconnect();
}

// Callback function for changing parameters dynamically
void SVHNode::dynamic_reconfigure_callback(svh_controller::svhConfig& config, uint32_t level)
{
  serial_device_name_ = config.serial_device;

  fm_->setResetSpeed(config.finger_reset_speed);
  fm_->setResetTimeout(config.reset_timeout);
}

// Callback function for connecting to SCHUNK five finger hand
void SVHNode::connectCallback(const std_msgs::Empty&)
{
  if (fm_->isConnected())
  {
    fm_->disconnect();
  }

  if (!fm_->connect(serial_device_name_, connect_retry_count))
  {
    ROS_ERROR(
      "Could not connect to SCHUNK five finger hand with serial device %s, and retry count %i",
      serial_device_name_.c_str(),
      connect_retry_count);
  }
}

// Callback function to reset/home channels of SCHUNK five finger hand
void SVHNode::resetChannelCallback(const std_msgs::Int8ConstPtr& channel)
{
  // convert int8 channel into SVHChannel enum
  driver_svh::SVHChannel svh_channel = static_cast<driver_svh::SVHChannel>(channel->data);

  if (fm_->resetChannel(svh_channel))
  {
    ROS_INFO("Channel %i successfully homed!", svh_channel);
  }
  else
  {
    ROS_ERROR("Could not reset channel %i !", svh_channel);
  }
}

// Callback function to enable channels of SCHUNK five finger hand
void SVHNode::enableChannelCallback(const std_msgs::Int8ConstPtr& channel)
{
  fm_->enableChannel(static_cast<driver_svh::SVHChannel>(channel->data));
}

// Callback function for setting channel target positions to SCHUNK five finger hand
void SVHNode::jointStateCallback(const sensor_msgs::JointStateConstPtr& input)
{
  // vector to read target positions from joint states
  std::vector<double> target_positions(driver_svh::eSVH_DIMENSION, 0.0);
  // bool vector storing true, if new target position read
  std::vector<bool> pos_read(driver_svh::eSVH_DIMENSION, false);
  // target positions read counter
  uint8_t pos_read_counter = 0;

  size_t index = 0;
  std::vector<std::string>::const_iterator joint_name;
  for (joint_name = input->name.begin(); joint_name != input->name.end(); ++joint_name, ++index)
  {
    int32_t channel = 0;

    // Find the corresponding channels to the input joint names
    bool valid_input = false;
    for (channel = 0; !valid_input && (channel < driver_svh::eSVH_DIMENSION) &&
                      (driver_svh::SVHController::m_channel_description[channel] != NULL);
         ++channel)
    {
      valid_input =
        (joint_name->compare(name_prefix + "_" +
                             driver_svh::SVHController::m_channel_description[channel]) == 0);
    }

    // We count one to high with the for loop so we have to correct that
    --channel;


    if (valid_input) //(icl_core::string2Enum((*joint_name), channel,
                     //driver_svh::SVHController::m_channel_description))
    {
      if (input->position.size() > index)
      {
        target_positions[channel] = input->position[index];
        pos_read[channel]         = true;
        pos_read_counter++;
      }
      else
      {
        ROS_WARN_STREAM("Vector of input joint state is too small! Cannot access element nr "
                        << index);
      }
    }
    else
    {
      // ROS_WARN("Could not map joint name %s to channel!", (*joint_name).c_str());
    }
  }

  // send target values at once
  if (pos_read_counter == driver_svh::eSVH_DIMENSION)
  {
    if (!fm_->setAllTargetPositions(target_positions))
    {
      ROS_WARN_ONCE("Set target position command rejected!");
    }
  }
  // not all positions has been set: send only the available positions
  else
  {
    for (size_t i = 0; i < driver_svh::eSVH_DIMENSION; ++i)
    {
      if (pos_read[i])
      {
        fm_->setTargetPosition(static_cast<driver_svh::SVHChannel>(i), target_positions[i], 0.0);
      }
    }
  }
}


sensor_msgs::JointState SVHNode::getChannelFeedback()
{
  if (fm_->isConnected())
  {
    // Get positions in rad
    for (size_t channel = 0; channel < driver_svh::eSVH_DIMENSION; ++channel)
    {
      double cur_pos = 0.0;
      double cur_cur = 0.0;
      if (fm_->isHomed(static_cast<driver_svh::SVHChannel>(channel)))
      {
        fm_->getPosition(static_cast<driver_svh::SVHChannel>(channel), cur_pos);
        // Read out currents if you want to
        fm_->getCurrent(static_cast<driver_svh::SVHChannel>(channel), cur_cur);
      }
      channel_pos_.position[channel] = cur_pos;
      channel_pos_.effort[channel] =
        cur_cur * driver_svh::SVHController::channel_effort_constants[channel];
    }
  }

  channel_pos_.header.stamp = ros::Time::now();
  return channel_pos_;
}

std_msgs::Float64MultiArray SVHNode::getChannelCurrents()
{
  if (fm_->isConnected())
  {
    // Get currents
    for (size_t channel = 0; channel < driver_svh::eSVH_DIMENSION; ++channel)
    {
      double cur_cur = 0.0;
      if (fm_->isHomed(static_cast<driver_svh::SVHChannel>(channel)))
      {
        fm_->getCurrent(static_cast<driver_svh::SVHChannel>(channel), cur_cur);
      }
      channel_currents.data[channel] = cur_cur;
    }
  }

  return channel_currents;
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
  ros::init(argc, argv, "svh_controller");
  // Private NH for general params
  ros::NodeHandle nh("~");


  // Tell ROS how fast to run this node. (100 = 100 Hz = 10 ms)
  ros::Rate rate(50);

  //==========
  // Logic
  //==========
  // Node object holding all the relevant functions
  SVHNode svh_node(nh);

  //==========
  // Dynamic Reconfigure
  //==========

  dynamic_reconfigure::Server<svh_controller::svhConfig> server;
  dynamic_reconfigure::Server<svh_controller::svhConfig>::CallbackType f;

  f = boost::bind(&SVHNode::dynamic_reconfigure_callback, &svh_node, _1, _2);
  server.setCallback(f);

  //==========
  // Callbacks
  //==========

  // Subscribe connect topic (Empty)
  ros::Subscriber connect_sub = nh.subscribe("connect", 1, &SVHNode::connectCallback, &svh_node);
  // Subscribe reset channel topic (Int8)
  ros::Subscriber reset_sub =
    nh.subscribe("reset_channel", 1, &SVHNode::resetChannelCallback, &svh_node);
  // Subscribe enable channel topic (Int8)
  ros::Subscriber enable_sub =
    nh.subscribe("enable_channel", 1, &SVHNode::enableChannelCallback, &svh_node);
  // Subscribe joint state topic
  ros::Subscriber channel_target_sub =
    nh.subscribe<sensor_msgs::JointState>("channel_targets",
                                          1,
                                          &SVHNode::jointStateCallback,
                                          &svh_node,
                                          ros::TransportHints().tcpNoDelay());
  // Publish current channel positions
  ros::Publisher channel_pos_pub = nh.advertise<sensor_msgs::JointState>("channel_feedback", 1);
  // Additionally publish just the current values of the motors
  ros::Publisher channel_current_pub =
    nh.advertise<std_msgs::Float64MultiArray>("channel_currents", 1);

  //==========
  // Messaging
  //==========

  // Main loop.
  while (nh.ok())
  {
    // get the current positions of all joints and publish them
    channel_pos_pub.publish(svh_node.getChannelFeedback());
    channel_current_pub.publish(svh_node.getChannelCurrents());

    ros::spinOnce();
    rate.sleep();
  }


  return 0;
}
