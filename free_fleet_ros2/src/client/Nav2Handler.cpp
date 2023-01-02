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

#include <deque>
#include <thread>
#include <optional>
#include <shared_mutex>

#include <rclcpp/rclcpp.hpp>
#include <tf2/impl/utils.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/msg/battery_state.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <geometry_msgs/msg/quaternion.hpp>
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

  geometry_msgs::msg::Quaternion quat_from_yaw(double yaw)
  {
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0.0, 0.0, yaw);
    quat_tf.normalize();

    return tf2::toMsg(quat_tf);
  }

  NavigateToPose::Goal nav_goal_from_location(const free_fleet::Location loc)
  {
    NavigateToPose::Goal nav_goal;
    nav_goal.pose.header.frame_id = map_frame;
    nav_goal.pose.pose.position.x = loc.x;
    nav_goal.pose.pose.position.y = loc.y;
    nav_goal.pose.pose.position.z = 0.0;
    nav_goal.pose.pose.orientation = quat_from_yaw(loc.yaw);
    return nav_goal;
  }

  rclcpp::Node::SharedPtr node;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr
    battery_state_sub;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client;
  std::shared_ptr<rclcpp::TimerBase> update_timer;

  rclcpp::CallbackGroup::SharedPtr callback_group;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor;

  mutable Mutex update_mutex;

  std::string robot_name;
  std::optional<std::string> task_id = std::nullopt;
  std::string map_name;
  std::optional<geometry_msgs::msg::PoseStamped> pose_stamped = std::nullopt;
  std::optional<double> speed = std::nullopt;
  std::optional<sensor_msgs::msg::BatteryState> battery_state = std::nullopt;

  std::string battery_state_topic = "battery_state";
  std::string map_frame = "map";
  std::string robot_frame = "baselink";
  double transform_timeout_secs = 0.1;

  std::thread follow_new_path_thread;
  Mutex follow_new_path_mutex;
  struct NavGoal
  {
    std::string level_name;
    NavigateToPose::Goal goal;
    std::optional<double> speed_limit = std::nullopt;
    bool sent = false;
    uint32_t aborted_count = 0;
  };
  std::shared_ptr<std::deque<NavGoal>> goal_path;

};

std::shared_ptr<Nav2Handler> Nav2Handler::make(
  rclcpp::Node::SharedPtr node,
  const std::string& robot_name,
  const std::string& map_frame,
  const std::string& robot_frame,
  const std::string& navigate_to_pose_server_name,
  const std::string& starting_map_name,
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
  handler->_pimpl->map_name = starting_map_name;
  handler->_pimpl->map_frame = map_frame;
  handler->_pimpl->robot_frame = robot_frame;
  handler->_pimpl->transform_timeout_secs = transform_timeout_secs;
  handler->_pimpl->node = std::move(node);
  handler->_pimpl->tf2_buffer = std::move(tf2_buffer);
  handler->_pimpl->tf2_listener = std::move(tf2_listener);
  handler->_pimpl->navigate_to_pose_client = std::move(navigate_to_pose_client);
  handler->_pimpl->battery_state_sub = std::move(battery_state_sub);
  handler->_pimpl->update_timer = std::move(update_timer);
  handler->_pimpl->goal_path =
    std::make_shared<std::deque<Implementation::NavGoal>>();

  handler->_pimpl->callback_group =
    handler->_pimpl->node->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
  handler->_pimpl->callback_group_executor.add_callback_group(
    handler->_pimpl->callback_group,
    handler->_pimpl->node->get_node_base_interface());
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
  ReadLock(_pimpl->update_mutex);

  nlohmann::json new_state;
  new_state["name"] = _pimpl->robot_name;

  auto& issues = new_state["issues"];
  issues = std::vector<nlohmann::json>();
  // TODO(aa): populate issues during operation.

  // TODO(aa): enum these values properly.
  // TODO(aa): to handle error, offline, shutdown.
  if (!_pimpl->battery_state.has_value())
  {
    new_state["status"] = "uninitialized";

    nlohmann::json issue_msg;
    issue_msg["category"] = "initialization";
    issue_msg["detail"] = "battery status not found.";
    issues.push_back(std::move(issue_msg));
  }
  else if (!_pimpl->speed.has_value())
  {
    new_state["status"] = "uninitialized";

    nlohmann::json issue_msg;
    issue_msg["category"] = "initialization";
    issue_msg["detail"] = "robot speed not initialized.";
    issues.push_back(std::move(issue_msg));
  }
  else if (_pimpl->battery_state->power_supply_status ==
    _pimpl->battery_state->POWER_SUPPLY_STATUS_CHARGING)
  {
    new_state["status"] = "charging";
  }
  else if (_pimpl->speed.value() < 1e-3)
  {
    new_state["status"] = "idle";
  }
  else
  {
    new_state["status"] = "moving";
  }

  new_state["task_id"] =
    _pimpl->task_id.has_value() ? _pimpl->task_id.value() : "";
  new_state["unix_millis_time"] =
    _pimpl->pose_stamped->header.stamp.sec * 1000 +
    static_cast<int>(_pimpl->pose_stamped->header.stamp.nanosec / 1000000);

  if (_pimpl->battery_state.has_value())
  {
    new_state["battery"] = _pimpl->battery_state->percentage;
  }

  nlohmann::json& location = new_state["location"];
  location["map"] = _pimpl->map_name;
  location["x"] = _pimpl->pose_stamped->pose.position.x;
  location["y"] = _pimpl->pose_stamped->pose.position.y;
  location["yaw"] =
    tf2::impl::getYaw(
      tf2::impl::toQuaternion(_pimpl->pose_stamped->pose.orientation));

  state = std::move(new_state);
  error.clear();
  return true;
}

bool Nav2Handler::relocalize(
  const free_fleet::Location& new_location,
  std::string& error)
{
  return false;
}

bool Nav2Handler::follow_new_path(
  const std::vector<free_fleet::Goal>& path,
  RequestCompleted path_finished_callback,
  std::string& error)
{
  // TODO: sanity check, first waypoint must be within N meters of our current
  // position. Otherwise, ignore the request.

  // check if we are currently doing any navigation
  // if so cancel all
  {
    WriteLock(_pimpl->follow_new_path_mutex);
    if (!_pimpl->goal_path->empty())
    {
      auto cancel_future =
        _pimpl->navigate_to_pose_client->async_cancel_all_goals();
      _pimpl->callback_group_executor.spin_until_future_complete(cancel_future);
      // for result callback processing
      _pimpl->callback_group_executor.spin_some();

      _pimpl->goal_path->clear();
    }

    // convert new path to goals
    for (const auto& goal : path)
    {
      _pimpl->goal_path->push_back(
        Implementation::NavGoal {
          goal.location.map_name,
          _pimpl->nav_goal_from_location(goal.location),
          goal.speed_limit,
          false,
          0});
    }
  }

  // start navigation


  // handle all this by spinning up another thread
  // auto send_goal_options =
  //   rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  // send_goal_options.goal_response_callback =
  //   [](std::shared_future<GoalHandleNavigateToPose::SharedPtr> future)
  //   {
  //     auto goal_handle = future.get();
  //     if (!goal_handle)
  //     {
  //       fferr << "Goal was rejected by server.\n";
  //     }
  //     else
  //     {
  //       ffinfo << "Goal accepted by server, waiting for result\n";
  //     }
  //   };
  // send_goal_options.feedback_callback =
  //   [](GoalHandleNavigateToPose::SharedPtr,
  //   const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  //   {
  //     ffinfo << "distace remaining: " << feedback->distance_remaining << "\n";
  //   };
  // send_goal_options.result_callback =
  //   [](const GoalHandleNavigateToPose::WrappedResult & result) {
  //     switch(result.code)
  //     {
  //       case rclcpp_action::ResultCode::SUCCEEDED:
  //         return;
  //       case rclcpp_action::ResultCode::ABORTED:
  //         return;
  //       case rclcpp_action::ResultCode::CANCELED:
  //         return;
  //       default:
  //         return;
  //     }
  //   };


  return false;
}

bool Nav2Handler::stop(std::string& error)
{
  return false;
}

bool Nav2Handler::resume(std::string& error)
{
  return false;
}

bool Nav2Handler::custom_command(
  const nlohmann::json& details,
  RequestCompleted custom_command_finished_callback,
  std::string& error)
{
  error = "No custom command supported.";
  return false;
}

} // namespace free_fleet_ros2
