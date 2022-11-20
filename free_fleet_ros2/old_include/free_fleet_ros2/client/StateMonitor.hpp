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

#ifndef INCLUDE__FREE_FLEET_ROS2__CLIENT__STATEMONITOR_HPP
#define INCLUDE__FREE_FLEET_ROS2__CLIENT__STATEMONITOR_HPP

#include <memory>

#include <rmf_utils/impl_ptr.hpp>
#include <free_fleet/client/StateMonitor.hpp>

namespace free_fleet_ros2 {

class StateMonitor : public free_fleet::StateMonitor
{
public:

  static std::shared_ptr<StateMonitor> make();

  bool current_state(
    nlohmann::json& state,
    std::string& error) const final;

  class Implementation;
private:
  StateMonitor();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

} // namespace free_fleet_ros2

#endif // INCLUDE__FREE_FLEET_ROS2__CLIENT__STATEMONITOR_HPP