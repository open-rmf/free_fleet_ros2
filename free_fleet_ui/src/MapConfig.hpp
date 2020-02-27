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

#ifndef FREE_FLEET_UI__SRC__MAPCONFIG_HPP
#define FREE_FLEET_UI__SRC__MAPCONFIG_HPP

#include <array>
#include <memory>

#include <QFileInfo>

namespace free_fleet
{
namespace viz
{

struct MapConfig
{

  QString image;
  double resolution;
  std::array<double, 3> origin;

  MapConfig();

  void print_config();

  using SharedPtr = std::shared_ptr<MapConfig>;

  static SharedPtr parse_map_config(const QFileInfo& config_file_info);
};

} // namespace viz
} // namespace free_fleet

#endif // FREE_FLEET_UI__SRC__MAPCONFIG_HPP
