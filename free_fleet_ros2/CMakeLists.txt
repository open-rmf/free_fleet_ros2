cmake_minimum_required(VERSION 3.7.0)
project(free_fleet_ros2)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

include(GNUInstallDirs)

find_package(ament_cmake REQUIRED)

set(dep_pkgs
  free_fleet
  free_fleet_mqtt
  geometry_msgs
  nav2_msgs
  nav2_util
  rclcpp
  rclcpp_action
  rmf_utils
  sensor_msgs
  std_srvs
  tf2
  tf2_ros
)
foreach(pkg ${dep_pkgs})
  find_package(${pkg} REQUIRED)
endforeach()

add_library(free_fleet_ros2 SHARED
  src/client/Nav2Handler.cpp
)

ament_target_dependencies(free_fleet_ros2
  ${dep_pkgs}
)

target_include_directories(free_fleet_ros2
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_executable(client
  src/client/main.cpp
)

target_link_libraries(client
  PRIVATE
    free_fleet_ros2
)

install(
  TARGETS
    free_fleet_ros2
    client
  RUNTIME DESTINATION lib/free_fleet_ros2
)

# install(DIRECTORY
#   launch/
#   DESTINATION share/${PROJECT_NAME}
# )

install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

ament_package()
