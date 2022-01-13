// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner exner@fzi.de
 * \date    2022-01-13
 *
 */
//----------------------------------------------------------------------
#pragma once

#include <string>

#include <ros/ros.h>

#include <schunk_svh_library/LogHandler.h>
#include <schunk_svh_library/LogLevel.h>

namespace driver_svh {

class ROSLogHandler : public LogHandler
{
public:
  ROSLogHandler()                   = default;
  virtual ~ROSLogHandler() override = default;
  virtual void log(const std::string& file,
                   const int line,
                   const std::string& name,
                   LogLevel level,
                   const std::string& msg) override;

private:
  static ::ros::console::Level levelSVH2ROS(const LogLevel level);
};

void setupROSLogHandler();

} // namespace driver_svh
