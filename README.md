# Free Fleet ROS2

## Contents

- **[About](#About)**
- **[Installation Instructions](#installation-instructions)**
  - [Prerequisites](#prerequisites)
- **[Examples](#examples)**
</br>
</br>

## About

Implementation of a Fleet Adapter to integrate [Free Fleet (an upcoming version)](https://github.com/open-rmf/free_fleet/tree/develop) clients with RMF [demos](https://github.com/open-rmf/rmf_demos).

</br>
</br>

## Installation Instructions

### Prerequisites

* [Ubuntu 20.04 LTS](https://releases.ubuntu.com/20.04/)
* [ROS2 - Galactic](https://docs.ros.org/en/galactic/index.html)
* [RMF demos](https://github.com/open-rmf/rmf_demos)
* [Free Fleet - develop branch](https://github.com/open-rmf/free_fleet/tree/develop)

</br>

### Installation
Assuming free_fleet and fleet_fleet_ros2 are to be located in your home directory.

#### Free Fleet (develop)

```
cd ~
git clone https://github.com/open-rmf/free_fleet.git -b develop
source /opt/ros/galactic/setup.bash
source ~/rmf_ws/install/setup.bash
cd ~/free_fleet
# free_fleet dependencies
colcon build
```

#### Free Fleet ROS2 Implementation
```bash
cd ~
git clone https://github.com/open-rmf/free_fleet_ros2.git -b develop
source /opt/ros/galactic/setup.bash
source ~/rmf_ws/install/setup.bash # requires rmf_fleet_msgs and rmf_task_msgs
source ~/free_fleet/install/setup.bash
colcon build
```
### Examples

To get the demos to use the free_fleet_ros2 adapter instead, modify this launch file:
Edit ~/rmf_ws/src/rmf/rmf_ros2/rmf_fleet_adapter/launch/fleet_adapter.launch.xml
Comment out lines 43-46
```xml
  <!-- <node pkg="rmf_fleet_adapter"
        exec="$(var control_type)"
        name="$(var fleet_name)_fleet_adapter"
        output="both"> -->
  <node pkg="free_fleet_ros2_adapter"
        exec="adapter"
        name="free_fleet_ros2_adapter"
        output="both">
```

Rebuild to use the updated launch file:
`cd ~/rmf_ws; colcon build --packages-select rmf_fleet_adapter`

Open one terminal for the demo+adapters and one for the clients.
In each:
```
source /opt/ros/galactic/setup.bash
source ~/rmf_ws/install/setup.bash
source ~/free_fleet_ros2/install/setup.bash

```
#### Launch demo
```
ros2 launch rmf_demos_gz hotel.launch.xml
```
#### Launch clients
The demo should be running before the clients are started.
```
ros2 launch free_fleet_ros2_examples hotel_clients.launch.xml
```

### To do
- During Cleaning and other tasks which are performed while docking, visualization will not work correctly.
- Support new speed limit in Location.
- Complete testing of opening and closing of lanes.
- Better approach to launch files and address upcoming changes to `rmf_demos` launch and config files.
- Launch files for remaining demo worlds.