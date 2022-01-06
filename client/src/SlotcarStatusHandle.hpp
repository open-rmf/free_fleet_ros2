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

#ifndef INCLUDE__FREE_FLEET_ROS2__CLIENT__SLOTCARSTATUSHANDLE__HPP
#define INCLUDE__FREE_FLEET_ROS2__CLIENT__SLOTCARSTATUSHANDLE__HPP

#include <free_fleet/client/StatusHandle.hpp>

#include "SlotcarCommandHandle.hpp"

class SlotcarStatusHandle : public free_fleet::client::StatusHandle
{
public:

  SlotcarStatusHandle(
    const SlotcarCommandHandle& command_handle
  );

  rmf_traffic::Time time() const;

  free_fleet::messages::Location location() const override final;

  free_fleet::messages::RobotMode mode() const override final;

  double battery_percent() const override final;

  std::optional<std::size_t> target_path_waypoint_index() const override final;

private:

  const SlotcarCommandHandle& _command_handle;

};

#endif // INCLUDE__FREE_FLEET_ROS2__CLIENT__SLOTCARSTATUSHANDLE__HPP
