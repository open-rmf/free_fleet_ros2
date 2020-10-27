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

#ifndef SRC__RMF_ADAPTER__FULLCONTROL_HPP
#define SRC__RMF_ADAPTER__FULLCONTROL_HPP

#include <memory>

#include <rmf_utils/impl_ptr.hpp>

#include <rmf_fleet_adapter/agv/RobotCommandHandle.hpp>

#include <free_fleet/transport/Middleware.hpp>

namespace free_fleet {
namespace rmf {

class FullControlHandle : public rmf_fleet_adapter::agv::RobotCommandHandle
{
public:

  using SharedPtr = std::shared_ptr<FullControlHandle>;
  
  FullControlHandle(
    rclcpp::Node& node,
    std::string fleet_name,
    std::string robot_name,
    std::shared_ptr<const rmf_traffic::agv::Graph> graph,
    std::shared_ptr<const rmf_traffic::agv::VehicleTraits> traits,
    std::shared_ptr<free_fleet::transport::Middleware> free_fleet_middleware);

  ~FullControlHandle();

  void follow_new_path(
    const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
    ArrivalEstimator next_arrival_estimator,
    RequestCompleted path_finished_callback) final;

  void stop() final;

  void dock(
    const std::string& dock_name,
    RequestCompleted docking_finished_callback) final;

  void set_updater(rmf_fleet_adapter::agv::RobotUpdateHandlePtr updater);

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // rmf
} // namespace free_fleet

#endif // SRC__RMF_ADAPTER__FULLCONTROL_HPP
