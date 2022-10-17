////////////////////////////////////////////////////////////////////////////////
//
// © Copyright 2022 SCHUNK Mobile Greifsysteme GmbH, Lauffen/Neckar Germany
// © Copyright 2022 FZI Forschungszentrum Informatik, Karlsruhe, Germany
//
// This file is part of the Schunk SVH Driver.
//
// The Schunk SVH Driver is free software: you can redistribute it and/or
// modify it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.
//
// The Schunk SVH Driver is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
// Public License for more details.
//
// You should have received a copy of the GNU General Public License along with
// Foobar. If not, see <https://www.gnu.org/licenses/>.
//
////////////////////////////////////////////////////////////////////////////////

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
