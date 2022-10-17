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
