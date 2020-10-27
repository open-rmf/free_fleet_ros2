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

#include <mutex>
#include <thread>
#include <iostream>

#include <free_fleet/messages/ModeRequest.hpp>
#include <free_fleet/messages/NavigationRequest.hpp>

#include <free_fleet_cyclonedds/CycloneDDSMiddleware.hpp>

#include <rmf_fleet_adapter/agv/Adapter.hpp>
#include <rmf_fleet_adapter/agv/parse_graph.hpp>

#include <rmf_traffic_ros2/Time.hpp>

#include "full_control.hpp"
#include "load_param.hpp"

namespace free_fleet {
namespace rmf {

//==============================================================================
class FullControlHandle::Implementation
{
public:

  using ArrivalEstimator =
    rmf_fleet_adapter::agv::RobotCommandHandle::ArrivalEstimator;
  using RequestCompleted =
    rmf_fleet_adapter::agv::RobotCommandHandle::RequestCompleted;

  rclcpp::Node* _node;

  std::vector<rmf_traffic::agv::Plan::Waypoint> _waypoints;
  ArrivalEstimator _next_arrival_estimator;
  RequestCompleted _path_finished_callback;
  rmf_utils::optional<std::size_t> _last_known_wp;
  rmf_fleet_adapter::agv::RobotUpdateHandlePtr _updater;
  std::shared_ptr<const rmf_traffic::agv::Graph> _graph;
  std::shared_ptr<const rmf_traffic::agv::VehicleTraits> _traits;
  std::shared_ptr<free_fleet::transport::Middleware> _free_fleet_middleware;

  std::string _fleet_name;
  std::string _robot_name;

  uint32_t _current_task_id = 0;

  std::mutex _mutex;
  std::thread _thread;

  Implementation()
  {}

  Implementation(const Implementation&)
  {}

  ~Implementation()
  {
    if (_thread.joinable())
      _thread.join();
  }

  void _loop()
  {
    while (_updater && _free_fleet_middleware)
    {
      // get state from free_fleet_middleware
      auto incoming_states = _free_fleet_middleware->read_states();

      // update using the updater
      for (const auto& rs : incoming_states)
      {
        if (!rs)
          continue;
      }

      // perform the current request lambda. This is for making sure the robot
      // executes the request, since most of the request functions are just
      // sending one single message.
    }
  }

  void _start()
  {
    RCLCPP_INFO(
      _node->get_logger(),
      "Starting Free Fleet full control handle.");
    _thread = std::thread(std::bind(&Implementation::_loop, this));
  }
};

//==============================================================================
FullControlHandle::FullControlHandle(
  rclcpp::Node& node,
  std::string fleet_name,
  std::string robot_name,
  std::shared_ptr<const rmf_traffic::agv::Graph> graph,
  std::shared_ptr<const rmf_traffic::agv::VehicleTraits> traits,
  std::shared_ptr<free_fleet::transport::Middleware> free_fleet_middleware)
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{
  _pimpl->_node = &node;
  _pimpl->_fleet_name = std::move(fleet_name);
  _pimpl->_robot_name = std::move(robot_name);
  _pimpl->_graph = std::move(graph);
  _pimpl->_traits = std::move(traits);
  _pimpl->_free_fleet_middleware = std::move(free_fleet_middleware);
}

//==============================================================================
FullControlHandle::~FullControlHandle()
{}

//==============================================================================
void FullControlHandle::follow_new_path(
  const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
  ArrivalEstimator next_arrival_estimator,
  RequestCompleted path_finished_callback)
{}

//==============================================================================
void FullControlHandle::stop()
{
  messages::ModeRequest request{
    _pimpl->_robot_name,
    std::to_string(_pimpl->_current_task_id++),
    messages::RobotMode{messages::RobotMode::MODE_PAUSED},
    {}};
  _pimpl->_free_fleet_middleware->send_mode_request(request);
}

//==============================================================================
void FullControlHandle::dock(
  const std::string& dock_name,
  RequestCompleted docking_finished_callback)
{}

//==============================================================================
void FullControlHandle::set_updater(
  rmf_fleet_adapter::agv::RobotUpdateHandlePtr updater)
{
  _pimpl->_updater = std::move(updater);
  _pimpl->_start();
}

//==============================================================================
} // rmf
} // namespace free_fleet

struct Connections : public std::enable_shared_from_this<Connections>
{
  /// The API for adding new robots to the adapter
  rmf_fleet_adapter::agv::FleetUpdateHandlePtr fleet;

  /// The API for running the fleet adapter
  rmf_fleet_adapter::agv::AdapterPtr adapter;

  /// The navigation graph for the robot
  std::shared_ptr<const rmf_traffic::agv::Graph> graph;

  /// The traits of the vehicles
  std::shared_ptr<const rmf_traffic::agv::VehicleTraits> traits;

  /// The container for robot update handles
  std::unordered_map<std::string, 
    std::shared_ptr<free_fleet::rmf::FullControlHandle>> robots;

  /// Middleware for free fleet
  std::shared_ptr<free_fleet::transport::Middleware> free_fleet_middleware;
  
  std::mutex mutex;

  void add_robot(
    const std::string& fleet_name,
    const free_fleet::messages::RobotState& state)
  {
    const auto& robot_name = state.name;
    const auto command = std::make_shared<free_fleet::rmf::FullControlHandle>(
      *adapter->node(),
      fleet_name,
      robot_name,
      graph,
      traits,
      free_fleet_middleware);

    const auto& loc = state.location;
    fleet->add_robot(
      command,
      robot_name,
      traits->profile(),
      rmf_traffic::agv::compute_plan_starts(
        *graph,
        state.location.level_name,
        {loc.x, loc.y, loc.y},
        rmf_traffic_ros2::convert(adapter->node()->now())),
      [c = weak_from_this(), command, robot_name = std::move(robot_name)](
        const rmf_fleet_adapter::agv::RobotUpdateHandlePtr& updater)
    {
      const auto connections = c.lock();
      if (!connections)
        return;

      std::lock_guard<std::mutex> lock(connections->mutex);

      command->set_updater(updater);
      connections->robots[robot_name] = command;
    });
  }
};

//==============================================================================
std::shared_ptr<Connections> make_fleet(
  const rmf_fleet_adapter::agv::AdapterPtr& adapter)
{
  const auto& node = adapter->node();
  std::shared_ptr<Connections> connections = std::make_shared<Connections>();
  connections->adapter = adapter;

  const std::string dds_domain_id_param_name = "dds_domain";
  const int dds_domain = node->declare_parameter(
    dds_domain_id_param_name, -1);
  if (dds_domain == -1)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Missing [%s] parameter", dds_domain_id_param_name.c_str());
    return nullptr;
  }

  const std::string fleet_name_param_name = "fleet_name";
  const std::string fleet_name = node->declare_parameter(
    fleet_name_param_name, std::string());
  if (fleet_name.empty())
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Missing [%s] parameter", fleet_name_param_name.c_str());
    return nullptr;
  }

  connections->traits =
    std::make_shared<rmf_traffic::agv::VehicleTraits>(
      free_fleet::rmf::get_traits_or_default(
        *node, 0.7, 0.3, 0.5, 1.5, 0.5, 1.5));

  const std::string nav_graph_param_name = "nav_graph_file";
  const std::string graph_file =
    node->declare_parameter(nav_graph_param_name, std::string());
  if (graph_file.empty())
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Missing [%s] parameter", nav_graph_param_name.c_str());
    return nullptr;
  }

  connections->graph =
    std::make_shared<rmf_traffic::agv::Graph>(
      rmf_fleet_adapter::agv::parse_graph(graph_file, *connections->traits));

  std::cout << "The fleet [" << fleet_name
            << "] has the following named waypoints:\n";
  for (const auto& key : connections->graph->keys())
    std::cout << " -- " << key.first << std::endl;

  connections->fleet = adapter->add_fleet(
    fleet_name, *connections->traits, *connections->graph);

  // If the perform_deliveries parameter is true, then we just blindly accept
  // all delivery requests.
  if (node->declare_parameter<bool>("perform_deliveries", false))
  {
    connections->fleet->accept_delivery_requests(
      [](const rmf_task_msgs::msg::Delivery&){ return true; });
  }

  if (node->declare_parameter<bool>("disable_delay_threshold", false))
  {
    connections->fleet->default_maximum_delay(rmf_utils::nullopt);
  }
  else
  {
    connections->fleet->default_maximum_delay(
      free_fleet::rmf::get_parameter_or_default_time(
        *node, "delay_threshold", 10.0));
  }

  connections->free_fleet_middleware =
    free_fleet::cyclonedds::CycloneDDSMiddleware::make_server(
      dds_domain, fleet_name);

  return connections;
}

//==============================================================================
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  const auto adapter = rmf_fleet_adapter::agv::Adapter::make("fleet_adapter");
  if (!adapter)
    return 1;

  const auto fleet_connections = make_fleet(adapter);
  if (!fleet_connections)
    return 1;
  
  RCLCPP_INFO(adapter->node()->get_logger(), "Starting Fleet Adapter");

  // Start running the adapter and wait until it gets stopped by SIGINT
  adapter->start().wait();

  RCLCPP_INFO(adapter->node()->get_logger(), "Closing Fleet Adapter");

  rclcpp::shutdown();
}
