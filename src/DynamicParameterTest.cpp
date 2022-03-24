// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Mauch <mauch@fzi.de>
 * \date    2017-2-22
 *
 */
//----------------------------------------------------------------------



#include <XmlRpcException.h>
#include <iostream>
#include <ros/ros.h>

#include "DynamicParameterClass.h"
#include <schunk_svh_library/control/SVHCurrentSettings.h>
#include <schunk_svh_library/control/SVHFingerManager.h>
#include <schunk_svh_library/control/SVHPositionSettings.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "DynamicParameterTest");
  ros::NodeHandle private_node("~");


  XmlRpc::XmlRpcValue parameters;
  private_node.getParam("VERSIONS_PARAMETERS", parameters);

  DynamicParameter dyn_parameters(2, 2, parameters);


  for (size_t channel = 0; channel < driver_svh::SVH_DIMENSION; ++channel)
  {
    ROS_INFO_STREAM("using channel " << channel);

    if (dyn_parameters.getSettings().position_settings_given[channel])
    {
      ROS_INFO("DYN POSITION SETTINGS...");
      for (size_t j = 0; j < dyn_parameters.getSettings().position_settings[channel].size(); j++)
      {
        std::cout << dyn_parameters.getSettings().position_settings[channel][j] << " ";
      }
      std::cout << std::endl;
    }

    if (dyn_parameters.getSettings().position_settings_given[channel])
    {
      ROS_INFO("DYN CURRENT SETTINGS:");
      for (size_t j = 0; j < dyn_parameters.getSettings().current_settings[channel].size(); j++)
      {
        std::cout << dyn_parameters.getSettings().current_settings[channel][j] << " ";
      }
      std::cout << std::endl;
    }

    if (dyn_parameters.getSettings().home_settings_given[channel])
    {
      ROS_INFO("DYN HOME SETTINGS:");
      for (size_t j = 0; j < dyn_parameters.getSettings().home_settings[channel].size(); j++)
      {
        std::cout << dyn_parameters.getSettings().home_settings[channel][j] << " ";
      }
    }
    std::cout << std::endl;
  }
  return 0;
}
