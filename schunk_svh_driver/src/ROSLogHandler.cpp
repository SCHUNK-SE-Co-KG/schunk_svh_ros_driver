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
// the Schunk SVH Driver. If not, see <https://www.gnu.org/licenses/>.
//
////////////////////////////////////////////////////////////////////////////////

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner mauch@fzi.de
 * \date    2022-01-13
 *
 */
//----------------------------------------------------------------------

#include "schunk_svh_driver/ROSLogHandler.hpp"

#include <schunk_svh_library/LogLevel.h>
#include <schunk_svh_library/Logger.h>

#include <memory>
#include <regex>

namespace driver_svh
{
auto g_log_handler = std::make_unique<ROSLogHandler>();

void ROSLogHandler::log(
  [[maybe_unused]] const std::string & file, [[maybe_unused]] const int line,
  const std::string & name, LogLevel level, const std::string & msg)
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
