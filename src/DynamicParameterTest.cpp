#include <XmlRpcException.h>
#include <iostream>
#include <ros/ros.h>

#include "DynamicParameterClass.h"
#include <driver_svh/SVHCurrentSettings.h>
#include <driver_svh/SVHFingerManager.h>
#include <driver_svh/SVHPositionSettings.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "DynamicParameterTest");
  ros::NodeHandle private_node("~");


  XmlRpc::XmlRpcValue parameters;
  private_node.getParam("VERSIONS_PARAMETERS", parameters);

  DynamicParameter dyn_parameters(2, 3, parameters);


  for (size_t channel = 0; channel < driver_svh::eSVH_DIMENSION; ++channel)
  {
    ROS_INFO("using channel %d", channel);

    if (dyn_parameters.m_position_settings_given[channel])
    {
      ROS_INFO("DYN POSITION SETTINGS...");
      for (int j = 0; j < dyn_parameters.m_position_settings[channel].size(); j++)
      {
        std::cout << dyn_parameters.m_position_settings[channel][j] << " ";
      }
      std::cout << std::endl;
    }

    if (dyn_parameters.m_position_settings_given[channel])
    {
      ROS_INFO("DYN CURRENT SETTINGS:");
      for (int j = 0; j < dyn_parameters.m_current_settings[channel].size(); j++)
      {
        std::cout << dyn_parameters.m_current_settings[channel][j] << " ";
      }
      std::cout << std::endl;
    }

    if (dyn_parameters.m_home_settings_given[channel])
    {
      ROS_INFO("DYN HOME SETTINGS:");
      for (int j = 0; j < dyn_parameters.m_home_settings[channel].size(); j++)
      {
        std::cout << dyn_parameters.m_home_settings[channel][j] << " ";
      }
    }
    std::cout << std::endl;
  }
  return 0;
}
