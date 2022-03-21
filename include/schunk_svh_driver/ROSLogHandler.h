// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    ROSLogHandler.h
 *
 * \author  Felix Exner exner@fzi.de
 * \date    2022-01-13
 *
 */
//-----------------------------------------------------------------------------

#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp/logger.hpp"

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
};

void setupROSLogHandler(LogLevel level = LogLevel::INFO);

} // namespace driver_svh
