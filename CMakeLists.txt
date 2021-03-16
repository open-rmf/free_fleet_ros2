cmake_minimum_required(VERSION 3.5.0)

project(free_fleet_ros2)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address")
# set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address")

find_package(ament_cmake REQUIRED)

include(GNUInstallDirs)

# if(BUILD_TESTING)
#   find_package(rmf_cmake_uncrustify REQUIRED)
#   find_file(uncrustify_config_file NAMES "share/format/rmf_code_style.cfg")

#   rmf_uncrustify(
#     ARGN include src test
#     CONFIG_FILE ${uncrustify_config_file}
#     MAX_LINE_LENGTH 80
#   )
# endif()

set(dep_pkgs
  rclcpp
  rmf_utils
  rmf_traffic
  rmf_fleet_msgs
  # rmf_fleet_adapter
  free_fleet
  free_fleet_cyclonedds
)
foreach(pkg ${dep_pkgs})
  find_package(${pkg} REQUIRED)
endforeach()

# ------------------------------------------------------------------------------

# file(GLOB lib_srcs
#   "src/agv/*.cpp"
# )

# add_library(free_fleet_ros2 SHARED
#   ${lib_srcs}
# )

# target_link_libraries(free_fleet_ros2
#   PRIVATE
#     ${rclcpp_LIBRARIES}
#     rmf_utils::rmf_utils
#     rmf_traffic::rmf_traffic
#     ${rmf_fleet_adapter_LIBRARIES}
#     free_fleet::free_fleet
#     free_fleet_cyclonedds::free_fleet_cyclonedds
# )

# target_include_directories(free_fleet_ros2
#   PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
#   PRIVATE
#     ${rclcpp_INCLUDE_DIRS}
#     ${rmf_fleet_adapter_INCLUDE_DIRS}
# )

# ------------------------------------------------------------------------------

# add_executable(full_control_adapter
#   "src/rmf_adapter/load_param.cpp"
#   "src/rmf_adapter/full_control.cpp"
# )

# target_link_libraries(full_control_adapter
#   PRIVATE
#     ${rclcpp_LIBRARIES}
#     rmf_utils::rmf_utils
#     rmf_traffic::rmf_traffic
#     rmf_traffic_ros2::rmf_traffic_ros2
#     rmf_fleet_adapter::rmf_fleet_adapter
#     free_fleet::free_fleet
#     free_fleet_cyclonedds::free_fleet_cyclonedds
# )

# target_include_directories(full_control_adapter
#   PRIVATE
#     ${rclcpp_INCLUDE_DIRS}
# )

# ------------------------------------------------------------------------------

# add_executable(traffic_light_adapter
#   "src/rmf_adapter/traffic_light.cpp"
# )

# target_link_libraries(traffic_light_adapter
#   PRIVATE
#     # free_fleet_ros2
#     ${rclcpp_LIBRARIES}
# )

# target_include_directories(traffic_light_adapter
#   PRIVATE
#     ${rclcpp_INCLUDE_DIRS}
# )

# ------------------------------------------------------------------------------

add_executable(minimal_server
  "src/MinimalServer.cpp"
)

target_link_libraries(minimal_server
  PRIVATE
    free_fleet::free_fleet
    free_fleet_cyclonedds::free_fleet_cyclonedds
    rmf_traffic::rmf_traffic
    ${rmf_fleet_msgs_LIBRARIES}
    ${rclcpp_LIBRARIES}
)
target_include_directories(minimal_server
  PRIVATE
    ${rmf_fleet_msgs_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
)

# ------------------------------------------------------------------------------

install(
  TARGETS
    # free_fleet_ros2
    # full_control_adapter
    # traffic_light_adapter
    minimal_server
  RUNTIME DESTINATION lib/free_fleet_ros2
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
