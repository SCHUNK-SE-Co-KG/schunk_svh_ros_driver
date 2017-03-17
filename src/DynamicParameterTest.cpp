#include <ros/ros.h>
#include <iostream>
#include <XmlRpcException.h>

#include <driver_svh/SVHFingerManager.h>
#include <driver_svh/SVHPositionSettings.h>
#include <driver_svh/SVHCurrentSettings.h>
#include "DynamicParameterClass.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "DynamicParameterTest");
  ros::NodeHandle private_node("~");
 
  
  XmlRpc::XmlRpcValue parameters;
  private_node.getParam("VERSIONS_PARAMETERS", parameters);

  DynamicParameter dyn_parameters(2,1,parameters);
  
  ROS_INFO("DYN POSITION SETTINGS:");
  for( int i = 0; i < dyn_parameters.m_position_settings.size(); i++){
    std::cout << dyn_parameters.m_position_settings[i] << std::endl;
  }
  /*
  ROS_INFO("DYN CURRENT SETTINGS:");
  for( int i = 0; i < dyn_parameters.m_current_settings.size(); i++){
    ROS_INFO("%d",dyn_parameters.m_position_settings[i]);
  }

  ROS_INFO("DYN HOME SETTINGS:");
  for( int i = 0; i < dyn_parameters.m_home_settings.size(); i++){
    ROS_INFO("%d",dyn_parameters.m_position_settings[i]);
  }*/
  return 0;
}
