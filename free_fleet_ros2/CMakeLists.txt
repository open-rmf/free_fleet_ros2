cmake_minimum_required(VERSION 3.5.0)

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
  rclcpp
  rclcpp_action
  tf2
  tf2_ros
  nav2_util
  std_srvs
  sensor_msgs
  nav2_msgs
  geometry_msgs
  free_fleet
)
foreach(pkg ${dep_pkgs})
  find_package(${pkg} REQUIRED)
endforeach()

file(GLOB lib_srcs
  "src/client/*.cpp"
)

add_library(free_fleet_ros2 SHARED
  ${lib_srcs}
)

ament_target_dependencies(free_fleet_ros2
  ${dep_pkgs}
)

install(
  TARGETS
    free_fleet_ros2
  RUNTIME DESTINATION lib/free_fleet_ros2
)

# install(DIRECTORY
#   launch/
#   DESTINATION share/${PROJECT_NAME}
# )

# install(
#   DIRECTORY include/
#   DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
# )

ament_package()
