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

  DynamicParameter dyn_parameters(2,0,parameters);

  return 0;
}
