/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic_ros2/Time.hpp>

#include "load_param.hpp"

namespace free_fleet {
namespace rmf {

//==============================================================================
std::chrono::nanoseconds get_parameter_or_default_time(
  rclcpp::Node& node,
  const std::string& param_name,
  const double default_value)
{
  const double value =
    get_parameter_or_default(node, param_name, default_value);

  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double, std::ratio<1>>(value));
}

//==============================================================================
rmf_traffic::agv::VehicleTraits get_traits_or_default(rclcpp::Node& node,
  const double default_v_nom, const double default_w_nom,
  const double default_a_nom, const double default_alpha_nom,
  const double default_r_f, const double default_r_v)
{
  const double v_nom =
    get_parameter_or_default(node, "linear_velocity", default_v_nom);
  const double w_nom =
    get_parameter_or_default(node, "angular_velocity", default_w_nom);
  const double a_nom =
    get_parameter_or_default(node, "linear_acceleration", default_a_nom);
  const double b_nom =
    get_parameter_or_default(node, "angular_acceleration", default_alpha_nom);
  const double r_f =
    get_parameter_or_default(node, "footprint_radius", default_r_f);
  const double r_v =
    get_parameter_or_default(node, "vicinity_radius", default_r_v);
  const bool reversible =
    get_parameter_or_default(node, "reversible", true);

  if (!reversible)
    std::cout << " ===== We have an irreversible robot" << std::endl;

  auto traits = rmf_traffic::agv::VehicleTraits{
    {v_nom, a_nom},
    {w_nom, b_nom},
    rmf_traffic::Profile{
      rmf_traffic::geometry::make_final_convex<
        rmf_traffic::geometry::Circle>(r_f),
      rmf_traffic::geometry::make_final_convex<
        rmf_traffic::geometry::Circle>(r_v)
    }
  };

  traits.get_differential()->set_reversible(reversible);
  return traits;
}

//==============================================================================
} // namespace rmf
} // namespace free_fleet
