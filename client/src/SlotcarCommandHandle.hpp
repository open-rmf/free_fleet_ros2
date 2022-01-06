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

#ifndef INCLUDE__FREE_FLEET_ROS2__CLIENT__SLOTCARCOMMANDHANDLE__HPP
#define INCLUDE__FREE_FLEET_ROS2__CLIENT__SLOTCARCOMMANDHANDLE__HPP

#include <rclcpp/rclcpp.hpp>

// Fleet driver state/command messages
#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_fleet_msgs/msg/path_request.hpp>
#include <rmf_fleet_msgs/msg/mode_request.hpp>
// #include <rmf_fleet_msgs/srv/lift_clearance.hpp>
// #include <rmf_fleet_msgs/msg/lane_request.hpp>
// #include <rmf_fleet_msgs/msg/closed_lanes.hpp>

#include <rmf_traffic/Time.hpp>

#include <free_fleet/client/CommandHandle.hpp>
#include <free_fleet/messages/Location.hpp>
#include <free_fleet/messages/Waypoint.hpp>
#include <free_fleet/messages/RobotMode.hpp>

class SlotcarCommandHandle
  : public free_fleet::client::CommandHandle
{
public:

  using PathRequestPub =
    rclcpp::Publisher<rmf_fleet_msgs::msg::PathRequest>::SharedPtr;

  using ModeRequestPub =
    rclcpp::Publisher<rmf_fleet_msgs::msg::ModeRequest>::SharedPtr;

  SlotcarCommandHandle(
    rclcpp::Node& node,
    std::string fleet_name,
    std::string robot_name,
    PathRequestPub path_request_pub,
    ModeRequestPub mode_request_pub
  );

  void relocalize(
    const free_fleet::messages::Location& location,
    RequestCompleted relocalization_finished_callback) override;

  void follow_new_path(
    const std::vector<free_fleet::messages::Waypoint>& waypoints,
    RequestCompleted path_finished_callback) override;

  void stop(RequestCompleted stopped_callback) override;

  void resume(RequestCompleted resumed_callback) override;

  bool dock(
    const std::string& dock_name,
    RequestCompleted docking_finished_callback) override;

  free_fleet::messages::Location location() const;

  free_fleet::messages::RobotMode mode() const;
  double battery_percent() const;

  std::optional<std::size_t> target_path_waypoint_index()
  const;

  free_fleet::messages::Location location();
  free_fleet::messages::RobotMode robot_mode();
  double battery_percent();
  std::optional<std::size_t> target_path_waypoint_index();

private:

  rclcpp::Node* _node;

  PathRequestPub _path_request_pub;
  rmf_fleet_msgs::msg::PathRequest _current_path_request;
  std::chrono::steady_clock::time_point _path_requested_time;

  rmf_fleet_msgs::msg::ModeRequest _current_dock_request;
  std::chrono::steady_clock::time_point _dock_requested_time;
  RequestCompleted _path_finished_callback;
  RequestCompleted _dock_finished_callback;
  ModeRequestPub _mode_request_pub;

  rclcpp::Subscription<rmf_fleet_msgs::msg::FleetState>::SharedPtr
    _fleet_state_sub;

  uint32_t _current_task_id = 0;

  std::mutex _mutex;

  std::unique_lock<std::mutex> _lock()
  {
    std::unique_lock<std::mutex> lock(_mutex, std::defer_lock);
    while (!lock.try_lock())
    {
      // Intentionally busy wait
    }

    return lock;
  }

  void fleet_state_cb(
    const rmf_fleet_msgs::msg::FleetState::SharedPtr msg,
    const std::string fleet_name,
    const std::string robot_name);

  void _clear_last_command();

  // Robot status
  free_fleet::messages::Location _location;
  free_fleet::messages::RobotMode _mode;
  double _battery_percent;
  std::optional<std::size_t> _target_path_waypoint_index;

};

#endif // INCLUDE__FREE_FLEET_ROS2__CLIENT__SLOTCARCOMMANDHANDLE__HPP
