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

#include <thread>
#include <optional>
#include <shared_mutex>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/impl/utils.h>
#include <sensor_msgs/msg/battery_state.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <nav2_util/robot_utils.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <free_fleet/Console.hpp>
#include <free_fleet_ros2/client/Nav2Handler.hpp>

namespace free_fleet_ros2 {

using Mutex = std::shared_mutex;
using ReadLock = std::shared_lock<Mutex>;
using WriteLock = std::unique_lock<Mutex>;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose =
  rclcpp_action::ClientGoalHandle<NavigateToPose>;

class Nav2Handler::Implementation :
  public std::enable_shared_from_this<Nav2Handler::Implementation>
{
public:
  Implementation()
  {}

  Implementation(const Implementation&)
  {}

  // void init_subscriptions(int update_period_millis);

  rclcpp::Node::SharedPtr node;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr
    battery_state_sub;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client;
  std::shared_ptr<rclcpp::TimerBase> update_timer;

  Mutex update_mutex;

  std::string robot_name;
  // "uninitialized", "offline", "shutdown", "idle", "charging", "working", "error"
  std::optional<std::string> task_id = std::nullopt;
  std::string map_name;
  std::optional<geometry_msgs::msg::PoseStamped> pose_stamped = std::nullopt;
  std::optional<double> speed = 0.0;
  std::optional<sensor_msgs::msg::BatteryState> battery_state = std::nullopt;

  std::string battery_state_topic = "battery_state";
  std::string map_frame = "map";
  std::string robot_frame = "baselink";
  double transform_timeout_secs = 0.1;
};

// void Nav2Handler::Implementation::init_subscriptions(int update_period_millis)
// {
//   using BatteryState = sensor_msgs::msg::BatteryState;
//   battery_state_sub = node->create_subscription<BatteryState>(
//     battery_state_topic, rclcpp::SensorDataQoS(),
//     [w = weak_from_this()](BatteryState::UniquePtr msg)
//     {
//       auto handler_impl = w.lock();
//       if (!handler_impl)
//       {
//         fferr << "handler_impl is not available.\n";
//         return;
//       }

//       WriteLock(handler_impl->update_mutex);
//       handler_impl->battery_state = *msg;
//     });

//   update_timer = node->create_wall_timer(
//     std::chrono::milliseconds(update_period_millis),
//     [w = weak_from_this()]()
//     {
//       auto handler_impl = w.lock();
//       if (!handler_impl)
//       {
//         fferr << "handler_impl is not available.\n";
//         return;
//       }

//       geometry_msgs::msg::PoseStamped new_pose_stamped;
//       if (!nav2_util::getCurrentPose(
//         new_pose_stamped,
//         *handler_impl->tf2_buffer,
//         handler_impl->map_frame,
//         handler_impl->robot_frame,
//         handler_impl->transform_timeout_secs))
//       {
//         ffwarn << "unable to retrieve transform from ["
//           << handler_impl->robot_frame << "] to ["
//           << handler_impl->map_frame << "].\n";
//         return;
//       }

//       double current_speed = 0;
//       {
//         ReadLock(handler_impl->update_mutex);
//         double elapsed_sec =
//           (rclcpp::Time(new_pose_stamped.header.stamp) -
//           handler_impl->pose_stamped->header.stamp)
//           .seconds();
//         double distance = hypot(
//           new_pose_stamped.pose.position.x -
//           handler_impl->pose_stamped->pose.position.x,
//           new_pose_stamped.pose.position.y -
//           handler_impl->pose_stamped->pose.position.y,
//           new_pose_stamped.pose.position.z -
//           handler_impl->pose_stamped->pose.position.z);
//         current_speed = abs(distance / elapsed_sec);
//       }

//       WriteLock(handler_impl->update_mutex);
//       handler_impl->pose_stamped = std::move(new_pose_stamped);
//       handler_impl->speed = current_speed;
//     });
// }

std::shared_ptr<Nav2Handler> Nav2Handler::make(
  rclcpp::Node::SharedPtr node,
  const std::string& robot_name,
  const std::string& map_frame,
  const std::string& robot_frame,
  const std::string& navigate_to_pose_server_name,
  int update_period_millis,
  int init_timeout_millis,
  double transform_timeout_secs)
{
  if (!node)
  {
    fferr << "provided node cannot be null.\n";
    return nullptr;
  }

  auto tf2_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    node->get_node_base_interface(),
    node->get_node_timers_interface());
  tf2_buffer->setCreateTimerInterface(timer_interface);
  tf2_buffer->setUsingDedicatedThread(true);
  auto tf2_listener = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer);

  geometry_msgs::msg::PoseStamped new_pose_stamped;
  ffinfo << "Retrieving transfrom from [" << robot_frame << "] to ["
    << map_frame << "].\n";
  if (!nav2_util::getCurrentPose(
    new_pose_stamped,
    *tf2_buffer,
    map_frame,
    robot_frame,
    static_cast<double>(init_timeout_millis)))
  {
    fferr << "timed out trying to retrieve transform from [" << robot_frame
      << "] to [" << map_frame << "].\n";
    return nullptr;
  }

  std::shared_ptr<Nav2Handler> handler(new Nav2Handler());
  handler->_pimpl->pose_stamped = std::move(new_pose_stamped);

  auto navigate_to_pose_client = rclcpp_action::create_client<NavigateToPose>(
    node, navigate_to_pose_server_name);
  if (!navigate_to_pose_client->wait_for_action_server(
    std::chrono::milliseconds(init_timeout_millis)))
  {
    fferr << "timed out waiting for navigate_to_pose action server ["
      << navigate_to_pose_server_name << "].\n";
    return nullptr;
  }

  using BatteryState = sensor_msgs::msg::BatteryState;
  auto battery_state_sub = node->create_subscription<BatteryState>(
    "battery_state",
    rclcpp::SensorDataQoS(),
    [w = handler->weak_from_this()](BatteryState::UniquePtr msg)
    {
      auto handler = w.lock();
      if (!handler)
      {
        fferr << "handler is not available.\n";
        return;
      }

      WriteLock(handler->_pimpl->update_mutex);
      handler->_pimpl->battery_state = *msg;
    });

  auto update_timer = node->create_wall_timer(
    std::chrono::milliseconds(update_period_millis),
    [w = handler->weak_from_this()]()
    {
      auto handler = w.lock();
      if (!handler)
      {
        fferr << "handler is not available.\n";
        return;
      }

      geometry_msgs::msg::PoseStamped new_pose_stamped;
      if (!nav2_util::getCurrentPose(
        new_pose_stamped,
        *handler->_pimpl->tf2_buffer,
        handler->_pimpl->map_frame,
        handler->_pimpl->robot_frame,
        handler->_pimpl->transform_timeout_secs))
      {
        ffwarn << "unable to retrieve transform from ["
          << handler->_pimpl->robot_frame << "] to ["
          << handler->_pimpl->map_frame << "].\n";
        return;
      }

      double current_speed = 0;
      {
        ReadLock(handler->_pimpl->update_mutex);
        double elapsed_sec =
          (rclcpp::Time(new_pose_stamped.header.stamp) -
          handler->_pimpl->pose_stamped->header.stamp)
          .seconds();
        double distance = hypot(
          new_pose_stamped.pose.position.x -
          handler->_pimpl->pose_stamped->pose.position.x,
          new_pose_stamped.pose.position.y -
          handler->_pimpl->pose_stamped->pose.position.y,
          new_pose_stamped.pose.position.z -
          handler->_pimpl->pose_stamped->pose.position.z);
        current_speed = abs(distance / elapsed_sec);
      }

      WriteLock(handler->_pimpl->update_mutex);
      handler->_pimpl->pose_stamped = std::move(new_pose_stamped);
      handler->_pimpl->speed = current_speed;
    });

  handler->_pimpl->robot_name = robot_name;
  handler->_pimpl->map_frame = map_frame;
  handler->_pimpl->robot_frame = robot_frame;
  handler->_pimpl->transform_timeout_secs = transform_timeout_secs;
  handler->_pimpl->node = std::move(node);
  handler->_pimpl->tf2_buffer = std::move(tf2_buffer);
  handler->_pimpl->tf2_listener = std::move(tf2_listener);
  handler->_pimpl->navigate_to_pose_client = std::move(navigate_to_pose_client);
  handler->_pimpl->battery_state_sub = std::move(battery_state_sub);
  handler->_pimpl->update_timer = std::move(update_timer);
  return handler;
}

Nav2Handler::Nav2Handler()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{
}

bool Nav2Handler::current_state(
  nlohmann::json& state,
  std::string& error) const
{
  nlohmann::json new_state;
  new_state["name"] = "test_robot";
  new_state["status"] = "idle";
  new_state["task_id"] = "";
  new_state["unix_millis_time"] = 20;
  new_state["battery"] = 1.0;

  nlohmann::json& location = new_state["location"];
  location["map"] = "test_map";
  location["x"] = 1.0;
  location["y"] = 2.0;
  location["yaw"] = 3.0;

  auto& issues = new_state["issues"];

  state = new_state;
  error = "";
  return true;
}

bool Nav2Handler::relocalize(
  const free_fleet::Location& new_location,
  RequestCompleted relocalize_finished_callback,
  std::string& error)
{
  return false;
}

bool Nav2Handler::follow_new_path(
  const std::vector<free_fleet::Goal>& path,
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
