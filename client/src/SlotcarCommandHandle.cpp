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
// ROS2 utilities for rmf_traffic
#include <rmf_traffic_ros2/Time.hpp>

#include <rmf_fleet_adapter/StandardNames.hpp>

#include "SlotcarCommandHandle.hpp"

SlotcarCommandHandle::SlotcarCommandHandle(
  rclcpp::Node& node,
  std::string fleet_name,
  std::string robot_name,
  PathRequestPub path_request_pub,
  ModeRequestPub mode_request_pub)
: _node(&node),
  _path_request_pub(path_request_pub),
  _mode_request_pub(mode_request_pub),
  _location("None", {0.0, 0.0}, 0.0),
  _mode(free_fleet::messages::RobotMode::Mode::Idle),
  _battery_percent(1.0)
{

  std::function<void(std::shared_ptr<rmf_fleet_msgs::msg::FleetState>)> fn =
    std::bind(
    &SlotcarCommandHandle::fleet_state_cb, this, std::placeholders::_1,
    fleet_name,
    robot_name);

  _fleet_state_sub = _node->create_subscription<
    rmf_fleet_msgs::msg::FleetState>(
    rmf_fleet_adapter::FleetStateTopicName,
    rclcpp::SystemDefaultsQoS(),
    fn
    );

  _current_path_request.fleet_name = fleet_name;
  _current_path_request.robot_name = robot_name;

  _current_dock_request.fleet_name = fleet_name;
  _current_dock_request.robot_name = robot_name;
  _current_dock_request.mode.mode = _current_dock_request.mode.MODE_DOCKING;

  rmf_fleet_msgs::msg::ModeParameter p;
  p.name = "docking";
  _current_dock_request.parameters.push_back(std::move(p));

}

void SlotcarCommandHandle::relocalize(
  const free_fleet::messages::Location& location,
  RequestCompleted relocalization_finished_callback)
{
  RCLCPP_WARN(
    _node->get_logger(),
    "Relocalize is not supported.");
  relocalization_finished_callback();
  return;
}

void SlotcarCommandHandle::follow_new_path(
  const std::vector<free_fleet::messages::Waypoint>& waypoints,
  RequestCompleted path_finished_callback)
{
  auto lock = _lock();
  _clear_last_command();

  _path_finished_callback = std::move(path_finished_callback);
  _target_path_waypoint_index = 0;
  _current_path_request.task_id = std::to_string(++_current_task_id);
  _current_path_request.path.clear();

  for (const auto& wp : waypoints)
  {
    rmf_fleet_msgs::msg::Location location;

    const Eigen::Vector2d p = wp.location().coordinates();
    if (wp.wait_until())
      location.t = rmf_traffic_ros2::convert(*(wp.wait_until()));

    location.x = p.x();
    location.y = p.y();
    location.yaw = (float) wp.location().yaw().value_or(0);

    _current_path_request.path.emplace_back(std::move(location));
  }

  _path_requested_time = std::chrono::steady_clock::now();
  _path_request_pub->publish(_current_path_request);
}

void SlotcarCommandHandle::stop(RequestCompleted stopped_callback)
{
  return;
}

void SlotcarCommandHandle::resume(RequestCompleted resumed_callback)
{
  return;
}

bool SlotcarCommandHandle::dock(
  const std::string& dock_name,
  RequestCompleted docking_finished_callback)
{
  auto lock = _lock();
  _clear_last_command();

  _dock_finished_callback = std::move(docking_finished_callback);
  _current_dock_request.parameters.front().value = dock_name;
  _current_dock_request.task_id = std::to_string(++_current_task_id);

  _dock_requested_time = std::chrono::steady_clock::now();
  _mode_request_pub->publish(_current_dock_request);

  // Didn't log _dock_target_wp
  RCLCPP_INFO(
    _node->get_logger(),
    "Requesting robot [%s] of [%s] to dock",
    _current_path_request.robot_name.c_str(),
    _current_path_request.fleet_name.c_str()
  );

  return true;
}

void SlotcarCommandHandle::fleet_state_cb(
  const rmf_fleet_msgs::msg::FleetState::SharedPtr msg,
  const std::string fleet_name,
  const std::string robot_name)
{
  if (msg->name != fleet_name)
  {
    return;
  }
  for (const auto& state : msg->robots)
  {
    if (state.name == robot_name)
    {
      _location = free_fleet::messages::Location(
        state.location.level_name,
        {state.location.x, state.location.y},
        state.location.yaw);

      auto rm = free_fleet::messages::RobotMode::Mode::Undefined;
      std::string info;

      switch (state.mode.mode)
      {
        case rmf_fleet_msgs::msg::RobotMode::MODE_IDLE:
          rm = free_fleet::messages::RobotMode::Mode::Idle;
          break;
        case rmf_fleet_msgs::msg::RobotMode::MODE_CHARGING:
          rm = free_fleet::messages::RobotMode::Mode::Charging;
          break;
        case rmf_fleet_msgs::msg::RobotMode::MODE_MOVING:
          rm = free_fleet::messages::RobotMode::Mode::Moving;
          break;
        case rmf_fleet_msgs::msg::RobotMode::MODE_PAUSED:
          rm = free_fleet::messages::RobotMode::Mode::Paused;
          break;
        case rmf_fleet_msgs::msg::RobotMode::MODE_EMERGENCY:
          rm = free_fleet::messages::RobotMode::Mode::Emergency;
          break;
        case rmf_fleet_msgs::msg::RobotMode::MODE_GOING_HOME:
          rm = free_fleet::messages::RobotMode::Mode::Custom;
          info = "MODE_GOING_HOME";
          break;
        case rmf_fleet_msgs::msg::RobotMode::MODE_DOCKING:
          rm = free_fleet::messages::RobotMode::Mode::Docking;
          break;
        case rmf_fleet_msgs::msg::RobotMode::MODE_ADAPTER_ERROR:
          rm = free_fleet::messages::RobotMode::Mode::Error;
          break;
        case rmf_fleet_msgs::msg::RobotMode::MODE_CLEANING:
          rm = free_fleet::messages::RobotMode::Mode::Custom;
          info = "MODE_CLEANING";
          break;
      }

      _mode = free_fleet::messages::RobotMode(rm, info);

      _battery_percent = state.battery_percent / 100;

      if (_path_finished_callback)
      {
        // If we have a path_finished_callback, then the robot should be
        // following a path
        _target_path_waypoint_index = _current_path_request.path.size() -
          state.path.size();

        // There should not be a docking command happening
        assert(!_dock_finished_callback);

        if (state.task_id != _current_path_request.task_id)
        {
          // The robot has not received our path request yet
          const auto now = std::chrono::steady_clock::now();
          if (std::chrono::milliseconds(200) < now - _path_requested_time)
          {
            // We published the request a while ago, so we'll send it again in
            // case it got dropped.
            _path_requested_time = now;
            _path_request_pub->publish(_current_path_request);
          }
          return;
        }
        if (state.path.empty())
        {
          // When the state path is empty, that means the robot believes it has
          // arrived at its destination.
          RCLCPP_INFO(
            _node->get_logger(),
            "%s has completed its path",
            robot_name.c_str());
          assert(_path_finished_callback);
          _path_finished_callback();
          _target_path_waypoint_index = std::nullopt;
          _path_finished_callback = nullptr;
        }

      }
      else if (_dock_finished_callback)
      {
        const auto now = std::chrono::steady_clock::now();
        // If we have a _dock_finished_callback, then the robot should be docking
        if (state.task_id != _current_dock_request.task_id)
        {
          if (std::chrono::milliseconds(200) < now - _dock_requested_time)
          {
            // We published the request a while ago, so we'll send it again in
            // case it got dropped.
            _dock_requested_time = now;
            _mode_request_pub->publish(_current_dock_request);
          }

          return;
        }

        if (state.mode.mode != state.mode.MODE_DOCKING)
        {
          RCLCPP_INFO(
            _node->get_logger(),
            "%s has completed docking",
            robot_name.c_str());
          _dock_finished_callback();
          _dock_finished_callback = nullptr;

          return;
        }
      }
    }
  }
}

free_fleet::messages::Location SlotcarCommandHandle::location() const
{
  return _location;
}

free_fleet::messages::RobotMode SlotcarCommandHandle::mode() const
{
  return _mode;
}

double SlotcarCommandHandle::battery_percent() const
{
  return _battery_percent;
}

std::optional<std::size_t> SlotcarCommandHandle::target_path_waypoint_index()
const
{
  return _target_path_waypoint_index;
}

void SlotcarCommandHandle::_clear_last_command()
{
  _path_finished_callback = nullptr;
  _dock_finished_callback = nullptr;
}