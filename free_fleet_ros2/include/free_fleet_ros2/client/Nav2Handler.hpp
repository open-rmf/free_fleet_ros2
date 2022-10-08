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

#ifndef INLCUDE__FREE_FLEET_ROS2__CLIENT__NAV2HANDLER_HPP
#define INLCUDE__FREE_FLEET_ROS2__CLIENT__NAV2HANDLER_HPP

#include <memory>

#include <rmf_utils/impl_ptr.hpp>
#include <free_fleet/client/RobotHandler.hpp>

namespace free_fleet_ros2 {

class Nav2Handler : public free_fleet::RobotHandler
{
public:

  // TODO
  static std::unique_ptr<Nav2Handler> make();

  bool current_state(
    nlohmann::json& state,
    std::string& error) const final;

  bool relocalize(
    const free_fleet::Location& new_location,
    RequestCompleted relocalize_finished_callback,
    std::string& error) final;

  bool follow_new_path(
    const std::vector<free_fleet::Location>& path,
    RequestCompleted path_finished_callback,
    std::string& error) final;

  bool stop(
    RequestCompleted stopped_callback,
    std::string& error) final;

  bool resume(
    RequestCompleted resumed_callback,
    std::string& error) final;

  bool custom_command(
    const nlohmann::json& details,
    RequestCompleted custom_command_finished_callback,
    std::string& error) final;

  class Implementation;
private:
  Nav2Handler();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

} // namespace free_fleet_ros2

#endif // INLCUDE__FREE_FLEET_ROS2__CLIENT__NAV2HANDLER_HPP
