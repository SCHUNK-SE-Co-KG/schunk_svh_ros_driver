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
      if (dyn_parameters.m_current_settings_given[channel])
      {
        m_finger_manager->setCurrentSettings(
          static_cast<driver_svh::SVHChannel>(channel),
          driver_svh::SVHCurrentSettings(dyn_parameters.m_current_settings[channel]));
      }
      if (dyn_parameters.m_position_settings_given[channel])
      {
        m_finger_manager->setPositionSettings(
          static_cast<driver_svh::SVHChannel>(channel),
          driver_svh::SVHPositionSettings(dyn_parameters.m_position_settings[channel]));
      }
      if (dyn_parameters.m_home_settings_given[channel])
      {
        m_finger_manager->setHomeSettings(
          static_cast<driver_svh::SVHChannel>(channel),
          driver_svh::SVHHomeSettings(dyn_parameters.m_home_settings[channel]));
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
    m_hardware_interface.reset(
      new SVHRosControlHWInterface(m_pub_nh, m_finger_manager, m_name_prefix));
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


// void SVHRosControlNode::goalCB (actionlib::ServerGoalHandle<
// control_msgs::FollowJointTrajectoryAction > gh)
//{
//  ROS_INFO ("Executing Trajectory action");

//  /* TODO: Catch errors:
//   * - Joint not enabled
//   * - EmergencyStopState
//   * - Overwriting trajectory
//   * - invalid positions
//   */


//  if (gh.getGoal()->trajectory.joint_names.size() !=
//  gh.getGoal()->trajectory.points[0].positions.size())
//  {
//    ROS_ERROR_STREAM ("Number of given joint names (" <<
//    gh.getGoal()->trajectory.joint_names.size() <<
//      ") and joint states (" << gh.getGoal()->trajectory.points.size() << ") do not match!
//      Aborting goal!");
//    return;
//  }

//  if (m_has_goal)
//  {
//    ROS_WARN_STREAM ("Sent a new goal while another goal was still running. Depending on the " <<
//      "device configuration this will either result in a queuing or the current trajectory " <<
//      "will be overwritten."
//    );
//  }


//  gh.setAccepted();
//  m_has_goal = true;
//  m_traj_thread = boost::thread(&SVHRosControlNode::trajThread, this, gh);

//}

// void SVHRosControlNode::trajThread(actionlib::ServerGoalHandle<
// control_msgs::FollowJointTrajectoryAction >& gh)
//{
//  control_msgs::FollowJointTrajectoryActionResult result;
//  control_msgs::FollowJointTrajectoryActionFeedback feedback;
//  feedback.feedback.header = gh.getGoal()->trajectory.header;
//  result.header = gh.getGoal()->trajectory.header;

//  ros::Time start = ros::Time::now();
//  bool targets_reached = true;

//  for (size_t waypoint = 0; waypoint < gh.getGoal()->trajectory.points.size(); ++waypoint)
//  {
//    feedback.feedback.desired.positions.clear();
//    feedback.feedback.joint_names.clear();
//    feedback.feedback.actual.positions.clear();
//    for (size_t i = 0; i < gh.getGoal()->trajectory.joint_names.size(); ++i)
//    {

//      uint8_t nr = m_joint_name_mapping[gh.getGoal()->trajectory.joint_names[i]];
//      float pos =
//      boost::lexical_cast<float>(gh.getGoal()->trajectory.points[waypoint].positions[i]);
//      ROS_INFO_STREAM ("Joint " << static_cast<int>(nr) << ": " << pos);
//      SchunkPowerBallNode::Ptr node;
//      try
//      {
//        node = m_controller->getNode<SchunkPowerBallNode>(nr);
//      }
//      catch (const NotFoundException& e)
//      {
//        ROS_ERROR_STREAM ("One or more nodes could not be found in the controller. " << e.what());
//        result.result.error_code = -2;
//        result.result.error_string = e.what();
//        gh.setAborted(result.result);
//        return;
//      }
//      m_controller->getNode<SchunkPowerBallNode>(nr)->setTarget(pos);
//      feedback.feedback.desired.positions.push_back(pos);
//      feedback.feedback.joint_names.push_back(gh.getGoal()->trajectory.joint_names[i]);


//      pos = node->getTargetFeedback();
//      feedback.feedback.actual.positions.push_back(pos);
//    }
//    ros::Duration max_time = gh.getGoal()->goal_time_tolerance;

//    ros::Duration spent_time = start - start;
//    std::vector<bool> reached_vec;

//    // Give the brakes time to open up
//    sleep(1);

//    while ( spent_time < max_time || max_time.isZero() )
//    {
//      try {
//        m_controller->syncAll();
//      }
//      catch (const std::exception& e)
//      {
//        ROS_ERROR_STREAM (e.what());
//        gh.setAborted();
//        return;
//      }
//      usleep(10000);

//      bool waypoint_reached = true;
//      for (size_t i = 0; i < gh.getGoal()->trajectory.joint_names.size(); ++i)
//      {
//        uint8_t nr = m_joint_name_mapping[gh.getGoal()->trajectory.joint_names[i]];
//        SchunkPowerBallNode::Ptr node = m_controller->getNode<SchunkPowerBallNode>(nr);
//        waypoint_reached &= node->isTargetReached();
//        float pos = node->getTargetFeedback();
//  //       ROS_INFO_STREAM ("Node " << nr << " target reached: " << waypoint_reached <<
//  //         ", position is: " << pos
//  //       );
//        feedback.feedback.actual.time_from_start = spent_time;
//        feedback.feedback.actual.positions.at(i) = (pos);
//      }


//      gh.publishFeedback(feedback.feedback);
//      targets_reached = waypoint_reached;


//      if (waypoint_reached)
//      {
//        ROS_INFO_STREAM ("Waypoint " << waypoint <<" reached" );
//        break;
//      }
//      spent_time = ros::Time::now() - start;
//      if (spent_time > max_time && !max_time.isZero())
//      {
//        result.result.error_code = -5;
//        result.result.error_string = "Did not reach targets in specified time";
//        gh.setAborted();
//        m_has_goal = false;
//        return;
//      }
//      if (boost::this_thread::interruption_requested() )
//      {
//        ROS_ERROR ("Interruption requested");
//        m_has_goal = false;
//        return;
//      }
//    }
//  }

//  if (targets_reached)
//  {
//    ROS_INFO ("All targets reached" );
//    result.result.error_code = 0;
//    result.result.error_string = "All targets successfully reached";
//    gh.setSucceeded(result.result);
//  }
//  else
//  {
//    ROS_INFO ("Not all targets reached" );
//    result.result.error_code = -5;
//    result.result.error_string = "Did not reach targets in specified time";
//    gh.setAborted();
//  }
//  m_has_goal = false;
//}


// void SVHRosControlNode::cancelCB (actionlib::ServerGoalHandle<
// control_msgs::FollowJointTrajectoryAction > gh)
//{
//  m_is_enabled = false;
//  ROS_INFO ("Canceling Trajectory action");

//  m_traj_thread.interrupt();
//  ROS_INFO ("Stopped trajectory thread");

//  for (size_t i = 0; i < m_chain_handles.size(); ++i)
//  {
//    m_chain_handles[i]->quickStop();
//  }
//  ROS_INFO ("Device stopped");
//  sleep(1);
//  for (size_t i = 0; i < m_chain_handles.size(); ++i)
//  {
//    m_chain_handles[i]->enableNodes();
//    m_is_enabled = true;
//  }

//  control_msgs::FollowJointTrajectoryActionResult result;
//  result.result.error_code = 0;
//  result.result.error_string = "Trajectory preempted by user";

//  gh.setCanceled(result.result);
//}

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
    m_hardware_interface->read();
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


// bool SVHRosControlNode::quickStopNodes(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse&
// resp)
//{
//  m_is_enabled = false;
//  if (!m_use_ros_control)
//  {
//    m_traj_thread.interrupt();
//    ROS_INFO ("Stopped trajectory thread");
//  }

//  try
//  {
//    for (size_t i = 0; i < m_chain_handles.size(); ++i)
//    {
//      m_chain_handles[i]->quickStop();
//    }
//  }
//  catch (const ProtocolException& e)
//  {
//    ROS_ERROR_STREAM ( "Error while quick stopping nodes: " << e.what());
//  }
//  resp.success = true;
//  m_was_disabled = false;
//  m_controller_manager->getControllerByName(m_traj_controller_name)->stopRequest(ros::Time::now());
//  ROS_INFO ("Quick stopped all nodes. For reenabling, please use the enable_nodes service. Thank
//  you for your attention.");
//  return true;
//}

bool SVHRosControlNode::homeAllNodes(schunk_svh_driver::HomeAllRequest& req,
                                     schunk_svh_driver::HomeAllResponse& resp)
{
  resp.success = m_finger_manager->resetChannel(driver_svh::eSVH_ALL);
  return resp.success;
}


// bool SVHRosControlNode::homeNodesJointNames(schunk_svh_driver::HomeWithJointNamesRequest& req,
//                                            schunk_svh_driver::HomeWithJointNamesResponse& resp)
//{
//  schunk_svh_driver::HomeWithIDsRequest req_fwd;
//  schunk_svh_driver::HomeWithIDsResponse resp_fwd;
//  for (std::vector<std::string>::iterator it = req.joint_names.begin();
//       it != req.joint_names.end();
//  ++it)
//  {
//    if (m_joint_name_mapping.find(*it) != m_joint_name_mapping.end())
//    {
//      req_fwd.node_ids.push_back(m_joint_name_mapping[*it]);
//    }
//    else
//    {
//      ROS_ERROR_STREAM ("Could not find joint " << *it << ". No homing will be performed for this
//      joint.");
//    }
//  }
//  homeNodesCanIds (req_fwd, resp_fwd);
//  resp.success = resp_fwd.success;
//  return resp.success;
//}


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
