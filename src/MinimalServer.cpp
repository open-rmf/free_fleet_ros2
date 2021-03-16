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

#include <iostream>
#include <memory>
#include <chrono>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_options.hpp>

#include <rmf_traffic/agv/Graph.hpp>
#include <rmf_fleet_msgs/msg/fleet_state.hpp>

#include <free_fleet/Manager.hpp>
#include <free_fleet/transport/Middleware.hpp>
#include <free_fleet/CoordinateTransformer.hpp>
#include <free_fleet_cyclonedds/CycloneDDSMiddleware.hpp>

class MinimalServer : public rclcpp::Node
{
public:

  using SharedPtr = std::shared_ptr<MinimalServer>;

  static SharedPtr make()
  {
    std::shared_ptr<MinimalServer> node(new MinimalServer());

    std::string fleet_name = "fleet_name";
    int domain_id = -1;
    std::string fleet_state_topic = "fleet_state_topic";
    int update_frequency = 10;

    double scale = 1.0;
    double translation_x = 0.0;
    double translation_y = 0.0;
    double rotation_yaw = 0.0;

    auto start_time = std::chrono::steady_clock::now();
    auto end_time = std::chrono::steady_clock::now();
    while (
      std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time)
        .count() < 10 &&
      domain_id < 0)
    {
      rclcpp::spin_some(node);
      node->get_parameter("fleet_name", fleet_name);
      node->get_parameter("domain_id", domain_id);
      node->get_parameter("fleet_state_topic", fleet_state_topic);
      node->get_parameter("update_frequency", update_frequency);
      node->get_parameter("scale", scale);
      node->get_parameter("translation_x", translation_x);
      node->get_parameter("translation_y", translation_y);
      node->get_parameter("rotation_yaw", rotation_yaw);

      end_time = std::chrono::steady_clock::now();
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    if (domain_id < 0)
    {
      std::cout << "Jeepers, waiting for parameters timed out." << std::endl;
      return nullptr;
    }

    std::shared_ptr<rmf_traffic::agv::Graph> graph(new rmf_traffic::agv::Graph);

    auto middleware =
      free_fleet::cyclonedds::CycloneDDSMiddleware::make_server(
        domain_id, fleet_name);

    auto to_robot_transform =
      free_fleet::CoordinateTransformer::make(
        scale,
        translation_x,
        translation_y,
        rotation_yaw);

    free_fleet::Manager::TimeNow time_now_fn =
      [](){ return std::chrono::steady_clock::now(); };

    free_fleet::Manager::RobotUpdatedCallback cb =
      [node](const std::shared_ptr<free_fleet::agv::RobotInfo>& robot)
    {
      node->update_robot(std::move(robot));
    };

    auto manager = free_fleet::Manager::make(
      fleet_name,
      std::move(graph),
      std::move(middleware),
      std::move(to_robot_transform),
      std::move(time_now_fn),
      std::move(cb));
    if (!manager)
    {
      return nullptr;
    }

    auto fleet_state_pub =
      node->create_publisher<rmf_fleet_msgs::msg::FleetState>(
        fleet_state_topic, 10);
    node->fleet_state_pub = std::move(fleet_state_pub);
    node->fleet_name = fleet_name;
    node->manager = std::move(manager);
    node->update_frequency = update_frequency;
    return node;
  }

  MinimalServer()
  : Node(
      "minimal_server_node",
      rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true))
  {}

  void start()
  {
    manager->start(update_frequency);

    auto timer_callback = [&]() -> void
    {
      std::lock_guard<std::mutex> lock(mutex);
      rmf_fleet_msgs::msg::FleetState fs;
      fs.name = fleet_name;
      for (const auto it : robots)
      {
        fs.robots.push_back(it.second);
      }
      // std::cout << "Before pub" << std::endl;
      fleet_state_pub->publish(fs);
    };
    using namespace std::chrono_literals;
    timer = create_wall_timer(500ms, timer_callback);
  }

private:

  std::mutex mutex;

  std::string fleet_name;

  free_fleet::Manager::SharedPtr manager;

  std::unordered_map<std::string, rmf_fleet_msgs::msg::RobotState> robots;

  rclcpp::Publisher<rmf_fleet_msgs::msg::FleetState>::SharedPtr fleet_state_pub;

  rclcpp::TimerBase::SharedPtr timer;

  int update_frequency = 10;

  void update_robot(const std::shared_ptr<free_fleet::agv::RobotInfo>& robot)
  {
    if (!robot)
      return;
    
    std::lock_guard<std::mutex> lock(mutex);
    rmf_fleet_msgs::msg::RobotState state;
    state.name = robot->name();
    state.location.x = robot->state().location.x;
    state.location.y = robot->state().location.y;
    state.location.yaw = robot->state().location.yaw;
    robots[robot->name()] = state;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto minimal_server = MinimalServer::make();

  if (!minimal_server)
  {
    std::cout << "Yikes something went wrong" << std::endl;
    return 1;
  }

  minimal_server->start();
  rclcpp::spin(minimal_server);
  rclcpp::shutdown();
  return 0;
}
