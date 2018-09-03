// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Andreas hermann <hermann@fzi.de>
 * \date    2016-02-19
 *
 */
//----------------------------------------------------------------------


#include "control_msgs/FollowJointTrajectoryActionFeedback.h"
#include "ros/service_server.h"
#include "std_msgs/Int16MultiArray.h"

#include "icl_core_logging/Logging.h"
#include <SVHRosControlNode.h>

#include "DynamicParameterClass.h"


SVHRosControlNode::SVHRosControlNode()
  : m_priv_nh("~")
  ,
  //    m_action_server(m_priv_nh, "follow_joint_trajectory",
  //    boost::bind(&SVHRosControlNode::goalCB, this, _1),
  //    boost::bind(&SVHRosControlNode::cancelCB, this, _1), false),
  m_has_goal(false)
  , m_use_ros_control(false)
  , m_was_disabled(false)
  , m_is_enabled(false)
  , m_homing_active(false)
  , m_nodes_initialized(false)
{
  //==========
  // Params
  //==========

  bool autostart, use_internal_logging;
  int reset_timeout;
  std::vector<bool> disable_flags(driver_svh::eSVH_DIMENSION, false);
  // Config that contains the log stream configuration without the file names
  std::string logging_config_file;
  int frequency = 30;

  sensor_msgs::JointState joint_msg;

  // Parameters that depend on the hardware version of the hand
  XmlRpc::XmlRpcValue dynamic_parameters;

  try
  {
    m_priv_nh.param<bool>("autostart", autostart, false);
    m_priv_nh.param<bool>("use_ros_control", m_use_ros_control, true);
    m_priv_nh.param<bool>("use_internal_logging", use_internal_logging, false);
    m_priv_nh.param<std::string>("serial_device", m_serial_device_name, "/dev/ttyUSB0");
    // Note : Wrong values (like numerics) in the launch file will lead to a "true" value here
    m_priv_nh.getParam("disable_flags", disable_flags);
    m_priv_nh.param<int>("reset_timeout", reset_timeout, 5);
    m_priv_nh.getParam("logging_config", logging_config_file);
    m_priv_nh.param<std::string>("name_prefix", m_name_prefix, "left_hand");
    m_priv_nh.param<int>("connect_retry_count", m_connect_retry_count, 3);
    m_priv_nh.param<std::string>(
      "traj_controller_name", m_traj_controller_name, "pos_based_pos_traj_controller_hand");
    m_priv_nh.getParam("VERSIONS_PARAMETERS", dynamic_parameters);
  }
  catch (ros::InvalidNameException e)
  {
    ROS_ERROR("Parameter Error!");
  }

  // Tell the user what we are using
  ROS_INFO("Name prefix for this Hand was set to :%s", m_name_prefix.c_str());

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
    {
      ROS_INFO("Internal logging was activated, output will be written as configured in "
               "logging.xml (default to ~/.ros/log)");
    }
    else 
    {
      ROS_WARN("Internal logging was enabled but config file could not be read. Please make sure "
               "the provided path to the config file is correct.");
    }
    ROS_WARN("Waiting for logging to initialize...");
    ros::Duration(5.0).sleep();
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
  m_finger_manager.reset(new driver_svh::SVHFingerManager(disable_flags, reset_timeout));


  // Receives current Firmware Version
  // because some parameters depend on the version
  m_finger_manager->connect(m_serial_device_name, m_connect_retry_count);

  // reads out current Firmware Version
  driver_svh::SVHFirmwareInfo version_info = m_finger_manager->getFirmwareInfo();
  ROS_INFO("current Handversion %d.%d", version_info.version_major, version_info.version_minor);
  m_finger_manager->disconnect();


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
          driver_svh::SVHPositionSettings(dyn_parameters.getSettings().position_settings[channel]));
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
    ROS_ERROR("Parameter Error! While reading the controller settings. Will use default settings");
  }

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
    ROS_INFO("SVH Driver Ready, you will need to connect and reset the fingers before you can use "
             "the hand.");
  }


  // Bring up roscontrol:
  if (m_use_ros_control)
  {
//     m_hardware_interface.reset(
//       new SVHRosControlHWInterface(m_pub_nh, m_finger_manager, m_name_prefix));
    m_controller_manager.reset(
      new controller_manager::ControllerManager(m_hardware_interface.get(), m_pub_nh));


    initDevices();

    // m_controller_manager->getControllerByName(m_traj_controller_name)->startRequest(ros::Time::now());

    ros::spin();
  }
  else
  {
    ros::Rate loop_rate(frequency);

    std_msgs::Int16MultiArray currents;
    while (ros::ok())
    {
      ros::spinOnce();

      if (m_nodes_initialized)
      {
        joint_msg.position.clear();
        currents.data.clear();
        for (std::map<std::string, uint8_t>::iterator it = m_joint_name_mapping.begin();
             it != m_joint_name_mapping.end();
             ++it)
        {
          double joint_value;
          m_finger_manager->getPosition(driver_svh::SVHChannel(it->second), joint_value);
          joint_msg.position.push_back(joint_value);

          // Schunk nodes write currents into the torque_actual register
          m_finger_manager->getCurrent(driver_svh::SVHChannel(it->second), joint_value);
          currents.data.push_back(joint_value);
        }
        joint_msg.header.stamp = ros::Time::now();
        m_joint_pub.publish(joint_msg);

        m_current_pub.publish(currents);
      }
      loop_rate.sleep();
    }
  }
}


bool SVHRosControlNode::initDevicesCb(std_srvs::TriggerRequest& req,
                                      std_srvs::TriggerResponse& resp)
{
  initDevices();
  resp.success = true;
  return resp.success;
}


void SVHRosControlNode::initDevices()
{
  // initialize all nodes, by default this will start ProfilePosition mode, so we're good to enable
  // nodes
  m_finger_manager->enableChannel(driver_svh::eSVH_ALL);

  // Start interface (either action server or ros_control)

  if (m_use_ros_control)
  {
    m_ros_control_thread = boost::thread(&SVHRosControlNode::rosControlLoop, this);
  }
  else
  {
    //    for (size_t i = 0; i < m_chain_handles.size(); ++i)
    //    {
    //      try {
    //        m_chain_handles[i]->setupProfilePositionMode(m_ppm_config);
    //        m_chain_handles[i]->enableNodes(mode);
    //      }
    //      catch (const ProtocolException& e)
    //      {
    //        ROS_ERROR_STREAM ("Caught ProtocolException while enabling nodes from chain " <<
    //          m_chain_handles[i]->getName() << ". Nodes from this group won't be enabled.");
    //        continue;
    //      }
    //      ROS_INFO_STREAM ("Enabled nodes from chain " << m_chain_handles[i]->getName());
    //    }
    //    m_action_server.start();
  }

  // Create joint state publisher
  m_joint_pub   = m_pub_nh.advertise<sensor_msgs::JointState>("joint_states", 1);
  m_current_pub = m_pub_nh.advertise<std_msgs::Int16MultiArray>("joint_currents", 1);

  // services
  m_enable_service =
    m_priv_nh.advertiseService("enable_nodes", &SVHRosControlNode::enableNodes, this);

  m_home_service_all =
    m_priv_nh.advertiseService("home_reset_offset_all", &SVHRosControlNode::homeAllNodes, this);
  m_home_service_joint_names = m_priv_nh.advertiseService(
    "home_reset_offset_by_id", &SVHRosControlNode::homeNodesChannelIds, this);
  //  m_home_service_joint_names = m_priv_nh.advertiseService("home_reset_offset_by_name",
  //     &SVHRosControlNode::homeNodesJointNames, this);

  m_nodes_initialized = true;
}



void SVHRosControlNode::rosControlLoop()
{
  ros::Duration elapsed_time;
  ros::Time last_time    = ros::Time::now();
  ros::Time current_time = ros::Time::now();

  m_finger_manager->enableChannel(driver_svh::eSVH_ALL);

  m_is_enabled = true;

  size_t counter = 0;

  while (ros::ok() && !m_homing_active)
  {
    current_time = ros::Time::now();
    elapsed_time = current_time - last_time;
    last_time    = current_time;
    // Input
    m_hardware_interface->read(current_time, elapsed_time);
    sensor_msgs::JointState joint_msg = m_hardware_interface->getJointMessage();
    if (m_joint_pub)
    {
      m_joint_pub.publish(joint_msg);
    }
    if (m_hardware_interface->isFault() && m_is_enabled)
    {
      m_controller_manager->getControllerByName(m_traj_controller_name)
        ->stopRequest(ros::Time::now());
      ROS_ERROR("Some nodes are in FAULT state! No commands will be sent. Once the fault is "
                "removed, call the enable_nodes service.");
      m_is_enabled = false;
    }
    // Control
    if (m_was_disabled && m_is_enabled)
    {
      ROS_INFO("Recovering from FAULT state. Resetting controller");
      m_controller_manager->update(current_time, elapsed_time, true);
      m_was_disabled = false;
    }
    else if (m_is_enabled)
    {
      m_controller_manager->update(current_time, elapsed_time);
      /* Give the controller some time, otherwise it will send a 0 waypoint vector which
       * might lead into a following error, if joints are not at 0
       * TODO: Find a better solution for that.
       */
      if (counter > 20)
      {
        // Output
        m_hardware_interface->write(current_time, elapsed_time);
      }
    }

    //     node->printStatus();


    ++counter;
    usleep(10000);
  }

  ROS_INFO("Shutting down ros_control_loop thread.");
}

bool SVHRosControlNode::enableNodes(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& resp)
{
  m_finger_manager->resetChannel(driver_svh::eSVH_ALL);

  // m_controller_manager->getControllerByName(m_traj_controller_name)->startRequest(ros::Time::now());

  m_was_disabled = true;
  m_is_enabled   = true;
  resp.success   = true;
  return true;
}


bool SVHRosControlNode::homeAllNodes(schunk_svh_driver::HomeAllRequest& req,
                                     schunk_svh_driver::HomeAllResponse& resp)
{
  resp.success = m_finger_manager->resetChannel(driver_svh::eSVH_ALL);
  return resp.success;
}


bool SVHRosControlNode::homeNodesChannelIds(schunk_svh_driver::HomeWithChannelsRequest& req,
                                            schunk_svh_driver::HomeWithChannelsResponse& resp)
{
  for (std::vector<uint8_t>::iterator it = req.channel_ids.begin(); it != req.channel_ids.end();
       ++it)
  {
    m_finger_manager->resetChannel(static_cast<driver_svh::SVHChannel>(*it));
  }

  if (m_use_ros_control)
  {
    m_ros_control_thread = boost::thread(&SVHRosControlNode::rosControlLoop, this);
  }

  m_homing_active = false;
  m_was_disabled  = true;
  m_is_enabled    = true;
  resp.success    = true;
  return resp.success;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "schunk_chain");

  SVHRosControlNode my_schunk_node;

  return 0;
}

