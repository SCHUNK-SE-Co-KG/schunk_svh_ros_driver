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
  ROSCONSOLE_DEFINE_LOCATION(true, levelSVH2ROS(level), std::string(ROSCONSOLE_NAME_PREFIX) + name);
  if (ROS_UNLIKELY(__rosconsole_define_location__enabled))
  {
    ros::console::print(NULL,
                        __rosconsole_define_location__loc.logger_,
                        levelSVH2ROS(level),
                        file.c_str(),
                        line,
                        "",
                        "%s",
                        msg.c_str());
  }
}

::ros::console::Level ROSLogHandler::levelSVH2ROS(const LogLevel level)
{
  switch (level)
  {
    case LogLevel::DEBUG:
      return ::ros::console::levels::Debug;
    case LogLevel::INFO:
      return ::ros::console::levels::Info;
    case LogLevel::WARN:
      return ::ros::console::levels::Warn;
    case LogLevel::ERROR:
      return ::ros::console::levels::Error;
    case LogLevel::FATAL:
      return ::ros::console::levels::Fatal;
    default:
      throw std::invalid_argument("Illegal logging level");
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
