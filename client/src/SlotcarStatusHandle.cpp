/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "SlotcarStatusHandle.hpp"

SlotcarStatusHandle::SlotcarStatusHandle(
  const SlotcarCommandHandle& command_handle
)
: _command_handle(command_handle)
{
}

rmf_traffic::Time SlotcarStatusHandle::time() const
{
  return std::chrono::steady_clock::now();
}

free_fleet::messages::Location SlotcarStatusHandle::location() const
{
  return _command_handle.location();
}

free_fleet::messages::RobotMode SlotcarStatusHandle::mode() const
{
  return _command_handle.mode();
}

double SlotcarStatusHandle::battery_percent() const
{
  return _command_handle.battery_percent();
}

std::optional<std::size_t> SlotcarStatusHandle::target_path_waypoint_index()
const
{
  return _command_handle.target_path_waypoint_index();
}