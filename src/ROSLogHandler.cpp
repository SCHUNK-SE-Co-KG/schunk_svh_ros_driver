// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner mauch@fzi.de
 * \date    2022-01-13
 *
 */
//----------------------------------------------------------------------

#include <memory>
#include <regex>

#include <ros/ros.h>

#include <schunk_svh_library/LogLevel.h>
#include <schunk_svh_library/Logger.h>

#include "ROSLogHandler.h"

namespace driver_svh {
std::unique_ptr<ROSLogHandler> g_log_handler(new ROSLogHandler);
bool g_handler_set = false;

void ROSLogHandler::log(const std::string& file,
                        const int line,
                        const std::string& name,
                        LogLevel level,
                        const std::string& msg)
{
  switch (level)
  {
    case LogLevel::DEBUG:
      ROS_DEBUG_STREAM_NAMED(name, msg);
      break;
    case LogLevel::INFO:
      ROS_INFO_STREAM_NAMED(name, msg);
      break;
    case LogLevel::WARN:
      ROS_WARN_STREAM_NAMED(name, msg);
      break;
    case LogLevel::ERROR:
      ROS_ERROR_STREAM_NAMED(name, msg);
      break;
    case LogLevel::FATAL:
      ROS_FATAL_STREAM_NAMED(name, msg);
      break;
  }
}

void setupROSLogHandler()
{
  if (!g_handler_set)
  {
    Logger::setLogLevel(LogLevel::DEBUG);
    Logger::setLogHandler(std::move(g_log_handler));
  }
}
} // namespace driver_svh
