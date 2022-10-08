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

#include <free_fleet_ros2/client/StateMonitor.hpp>

namespace free_fleet_ros2 {

class StateMonitor::Implementation
{
public:
  // name
  // status
  // task id
  // location
  // battery
  // issues
};

std::shared_ptr<StateMonitor> StateMonitor::make()
{
  return nullptr;
}

StateMonitor::StateMonitor()
: _pimpl(rmf_utils::make_unique_impl<Implementation>())
{
}

bool StateMonitor::current_state(
  nlohmann::json& state,
  std::string& error) const
{
  return false;
}

} // namespace free_fleet_ros2
