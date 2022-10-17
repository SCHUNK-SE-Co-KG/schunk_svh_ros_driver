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

void setupROSLogHandler(LogLevel level)
{
  if (g_log_handler != nullptr) {
    Logger::setLogLevel(level);
    Logger::setLogHandler(std::move(g_log_handler));
  }
}
} // namespace driver_svh
