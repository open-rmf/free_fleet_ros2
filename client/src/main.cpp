#include <free_fleet/client/Client.hpp>
#include <free_fleet/Executor.hpp>
#include <free_fleet/Worker.hpp>
#include <free_fleet_cyclonedds/ClientDDSMiddleware.hpp>

#include "SlotcarCommandHandle.hpp"
#include "SlotcarStatusHandle.hpp"

int main(int argc, char** argv)
{

  rclcpp::init(argc, argv);
  using PathRequestPub =
    rclcpp::Publisher<rmf_fleet_msgs::msg::PathRequest>::SharedPtr;
  using ModeRequestPub =
    rclcpp::Publisher<rmf_fleet_msgs::msg::ModeRequest>::SharedPtr;
  const auto node = std::make_shared<rclcpp::Node>("free_fleet_client");

  const std::string fleet_name_param_name = "fleet_name";
  const std::string fleet_name = node->declare_parameter(
    fleet_name_param_name, std::string());
  if (fleet_name.empty())
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Missing [%s] parameter", fleet_name_param_name.c_str());

    return 1;
  }

  const std::string robot_name_param_name = "robot_name";
  const std::string robot_name = node->declare_parameter(
    robot_name_param_name, std::string());
  if (robot_name.empty())
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Missing [%s] parameter", robot_name_param_name.c_str());

    return 1;
  }

  const std::string robot_model_param_name = "robot_model";
  const std::string robot_model = node->declare_parameter(
    robot_model_param_name, std::string());
  if (robot_model.empty())
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Missing [%s] parameter", robot_model_param_name.c_str());

    return 1;
  }

  const std::string dds_domain_param_name = "dds_domain";
  node->declare_parameter(dds_domain_param_name);
  const int dds_domain = node->get_parameter(dds_domain_param_name).as_int();

  PathRequestPub path_request_pub = node->create_publisher<
    rmf_fleet_msgs::msg::PathRequest>(
    "robot_path_requests", rclcpp::SystemDefaultsQoS());

  ModeRequestPub mode_request_pub = node->create_publisher<
    rmf_fleet_msgs::msg::ModeRequest>(
    "robot_mode_requests", rclcpp::SystemDefaultsQoS());

  auto command_handle = std::make_shared<SlotcarCommandHandle>(
    *node,
    fleet_name,
    robot_name,
    path_request_pub,
    mode_request_pub);

  auto status_handle = std::make_shared<SlotcarStatusHandle>(
    *command_handle
  );

  auto middleware = free_fleet::cyclonedds::ClientDDSMiddleware::make_unique(
    dds_domain,
    fleet_name);

  auto client = free_fleet::Client::make(
    robot_name,
    robot_model,
    command_handle,
    status_handle,
    std::move(middleware)
  );

  free_fleet::Executor executor(
    (std::unique_ptr<free_fleet::Worker>)client.get());

  RCLCPP_INFO(
    node->get_logger(),
    "Starting free fleet client for %s",
    robot_name.c_str());
  executor.start_async(std::chrono::nanoseconds((int)1e9));

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;

}