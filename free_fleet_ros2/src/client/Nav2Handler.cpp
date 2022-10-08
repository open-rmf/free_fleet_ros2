/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include <free_fleet_ros2/client/Nav2Handler.hpp>

namespace free_fleet_ros2 {

class Nav2Handler::Implementation
{
public:
  //
};

std::unique_ptr<Nav2Handler> Nav2Handler::make()
{
  return nullptr;
}

Nav2Handler::Nav2Handler()
: _pimpl(rmf_utils::make_unique_impl<Implementation>())
{
}

bool Nav2Handler::current_state(
  nlohmann::json& state,
  std::string& error) const
{
  return false;
}

bool Nav2Handler::relocalize(
  const free_fleet::Location& new_location,
  RequestCompleted relocalize_finished_callback,
  std::string& error)
{
  return false;
}

bool Nav2Handler::follow_new_path(
  const std::vector<free_fleet::Location>& path,
  RequestCompleted path_finished_callback,
  std::string& error)
{
  return false;
}

bool Nav2Handler::stop(
  RequestCompleted stopped_callback,
  std::string& error)
{
  return false;
}

bool Nav2Handler::resume(
  RequestCompleted resumed_callback,
  std::string& error)
{
  return false;
}

bool Nav2Handler::custom_command(
  const nlohmann::json& details,
  RequestCompleted custom_command_finished_callback,
  std::string& error)
{
  return false;
}

} // namespace free_fleet_ros2
