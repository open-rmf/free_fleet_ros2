cmake_minimum_required(VERSION 3.7.0)
project(free_fleet_ros2_adapter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  # Use the Release build type by default if the user has not specified one
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
include(GNUInstallDirs)

set(dep_pkgs
  rclcpp
  rmf_fleet_msgs
  rmf_task_msgs
  rmf_traffic
  rmf_traffic_ros2
  rmf_fleet_adapter
  free_fleet
  free_fleet_cyclonedds
  CycloneDDS
)
foreach(pkg ${dep_pkgs})
  find_package(${pkg} REQUIRED)
endforeach()

# -----------------------------------------------------------------------------
add_executable(adapter 
  src/main.cpp
)

target_link_libraries(adapter
  PRIVATE
    free_fleet::free_fleet
    free_fleet_cyclonedds::free_fleet_cyclonedds
    rmf_fleet_adapter::rmf_fleet_adapter
    rmf_traffic::rmf_traffic
    ${rclcpp_LIBRARIES}
    ${rmf_fleet_msgs_LIBRARIES}
    ${rmf_task_msgs_LIBRARIES}
    CycloneDDS::ddsc
)
target_include_directories(adapter
  PRIVATE
    ${rmf_fleet_msgs_INCLUDE_DIRS}
    ${rmf_task_msgs_INCLUDE_DIRS}
)

install(TARGETS
  adapter
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()