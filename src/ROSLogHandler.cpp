////////////////////////////////////////////////////////////////////////////////
// Copyright 2022 FZI Research Center for Information Technology
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
