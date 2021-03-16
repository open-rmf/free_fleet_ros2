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

  static SharedPtr make(
    const std::string& fleet_name,
    std::shared_ptr<rmf_traffic::agv::Graph> graph,
    std::shared_ptr<free_fleet::transport::Middleware> middleware,
    std::shared_ptr<free_fleet::CoordinateTransformer> to_robot_transform,
    const std::string& fleet_state_topic)
  {
    std::shared_ptr<MinimalServer> node(new MinimalServer());
    auto fleet_state_pub = 
      node->create_publisher<rmf_fleet_msgs::msg::FleetState>(
        fleet_state_topic, 10);

    node->fleet_state_pub = std::move(fleet_state_pub);

    free_fleet::Manager::TimeNow time_now_fn =
      [](){ return std::chrono::steady_clock::now(); };
    free_fleet::Manager::RobotUpdatedCallback cb =
      [node](const std::shared_ptr<free_fleet::agv::RobotInfo>& robot)
    {
      node->update_robots(robot);
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

    node->fleet_name = fleet_name;
    node->manager = std::move(manager);
    return node;
  }

  MinimalServer()
  : Node("minimal_server_node")
  {}

  void start(uint32_t freq)
  {
    manager->start(freq);

    auto timer_callback = [this]() -> void
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      rmf_fleet_msgs::msg::FleetState fs;
      fs.name = this->fleet_name;
      for (const auto it : this->robots)
      {
        fs.robots.push_back(it.second);
      }
      this->fleet_state_pub->publish(fs);
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

  void update_robots(const std::shared_ptr<free_fleet::agv::RobotInfo>& robot)
  {
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
  std::string fleet_name = "test_fleet";
  std::shared_ptr<rmf_traffic::agv::Graph> graph(new rmf_traffic::agv::Graph);
  auto m =
    free_fleet::cyclonedds::CycloneDDSMiddleware::make_server(24, fleet_name);
  auto t =
    free_fleet::CoordinateTransformer::make(1.0, 0.0, 0.0, 0.0);
  auto minimal_server = MinimalServer::make(
    fleet_name,
    graph,
    m,
    t,
    "fleet_states");

  if (!minimal_server)
    std::cout << "yikes something went wrong" << std::endl;
  minimal_server->start(10);

  rclcpp::spin(minimal_server);

  std::cout << "All done" << std::endl;
  rclcpp::shutdown();
  return 0;
}
