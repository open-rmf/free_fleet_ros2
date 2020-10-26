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

#include "full_control.hpp"

namespace free_fleet {
namespace rmf {

//==============================================================================
class FullControlHandle::Implementation
{
public:

  std::shared_ptr<free_fleet::transport::Middleware> _middleware;
};

//==============================================================================
FullControlHandle::SharedPtr FullControlHandle::make(
  std::shared_ptr<free_fleet::transport::Middleware> middleware)
{
  SharedPtr handle(new FullControlHandle);
  handle->_pimpl->_middleware = std::move(middleware);
  return handle;
}

//==============================================================================
FullControlHandle::FullControlHandle()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{}

//==============================================================================
FullControlHandle::~FullControlHandle()
{}

//==============================================================================
void FullControlHandle::follow_new_path(
  const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
  ArrivalEstimator next_arrival_estimator,
  RequestCompleted path_finished_callback)
{}

//==============================================================================
void FullControlHandle::stop()
{}

//==============================================================================
void FullControlHandle::dock(
  const std::string& dock_name,
  RequestCompleted docking_finished_callback)
{}

//==============================================================================
//==============================================================================
} // rmf
} // namespace free_fleet

int main()
{
  std::cout << "all done" << std::endl;
  return 0;
}
