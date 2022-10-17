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
 * \date    2017-2-30
 *
 */
//----------------------------------------------------------------------


// ROS includes
#include <ros/ros.h>

// package includes
#include "SVHRosControlHWInterface.h"

// other includes

int main(int argc, char** argv)
{
  ros::init(argc, argv, "schunk_svh");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::NodeHandle nh("~");

  // Create the hardware interface
  SVHRosControlHWInterface svh_hw;
  svh_hw.init(nh, nh);

  controller_manager::ControllerManager cm(&svh_hw, nh);

  ros::Time timestamp_now  = ros::Time::now();
  ros::Time timestamp_last = ros::Time::now();
  ros::Duration period;

  ros::Rate rate(125);
  while (ros::ok())
  {
    // Get current time and elapsed time since last read
    timestamp_now  = ros::Time::now();
    period         = timestamp_now - timestamp_last;
    timestamp_last = timestamp_now;

    svh_hw.read(timestamp_now, period);
    cm.update(timestamp_now, period, !svh_hw.isEnabled());
    svh_hw.write(timestamp_now, period);
    rate.sleep();
  }

  return 0;
}
