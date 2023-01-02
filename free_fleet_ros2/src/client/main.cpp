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

#include <rclcpp/rclcpp.hpp>

#include <free_fleet/Console.hpp>
#include <free_fleet/Executor.hpp>
#include <free_fleet/client/Client.hpp>
#include <free_fleet_mqtt/MqttMiddleware.hpp>
#include <free_fleet_ros2/client/Nav2Handler.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("test_robot_free_fleet_client_ros2");

  std::shared_ptr<free_fleet::RobotHandler> handler =
    free_fleet_ros2::Nav2Handler::make(
      node,
      "test_robot",
      "map",
      "base_footprint",
      "navigate_to_pose",
      "L1");
  auto middleware = free_fleet::transport::MqttMiddleware::make(
    "tcp://localhost:1883",
    "test_free_fleet_mqtt__make_MqttMiddleware");
  auto client = free_fleet::Client::make(
    "new_robot",
    std::move(handler),
    std::move(middleware));
  free_fleet::Executor executor(std::move(client));

  executor.start_async(std::chrono::nanoseconds(500000000));
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
