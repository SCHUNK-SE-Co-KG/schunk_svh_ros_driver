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
