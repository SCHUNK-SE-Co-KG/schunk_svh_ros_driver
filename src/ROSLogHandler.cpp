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

#include "schunk_svh_driver/ROSLogHandler.h"

#include <schunk_svh_library/LogLevel.h>
#include <schunk_svh_library/Logger.h>

#include <memory>
#include <regex>

namespace driver_svh
{
auto g_log_handler = std::make_unique<ROSLogHandler>();

void ROSLogHandler::log(
  const std::string & file, const int line, const std::string & name, LogLevel level,
  const std::string & msg)
{
  switch (level) {
    case LogLevel::DEBUG:
      RCLCPP_DEBUG(rclcpp::get_logger(name), msg.c_str());
      break;
    case LogLevel::INFO:
      RCLCPP_INFO(rclcpp::get_logger(name), msg.c_str());
      break;
    case LogLevel::WARN:
      RCLCPP_WARN(rclcpp::get_logger(name), msg.c_str());
      break;
    case LogLevel::ERROR:
      RCLCPP_ERROR(rclcpp::get_logger(name), msg.c_str());
      break;
    case LogLevel::FATAL:
      RCLCPP_FATAL(rclcpp::get_logger(name), msg.c_str());
      break;
    default:
      break;
  }
}

void setupROSLogHandler(LogLevel level)
{
  if (g_log_handler != nullptr) {
    Logger::setLogLevel(level);
    Logger::setLogHandler(std::move(g_log_handler));
  }
}
}  // namespace driver_svh
