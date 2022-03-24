// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    ROSLogHandler.hpp
 *
 * \author  Felix Exner exner@fzi.de
 * \date    2022-01-13
 *
 */
//-----------------------------------------------------------------------------

#pragma once

#include <schunk_svh_library/LogHandler.h>
#include <schunk_svh_library/LogLevel.h>

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "rclcpp/logger.hpp"

namespace driver_svh
{
class ROSLogHandler : public LogHandler
{
public:
  ROSLogHandler() = default;
  virtual ~ROSLogHandler() override = default;
  virtual void log(
    const std::string & file, const int line, const std::string & name, LogLevel level,
    const std::string & msg) override;

private:
};

void setupROSLogHandler(LogLevel level = LogLevel::INFO);

}  // namespace driver_svh
