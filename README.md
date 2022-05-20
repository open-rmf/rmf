# RMF

![](https://github.com/open-rmf/rmf/workflows/build/badge.svg)

![Nightly](https://github.com/open-rmf/rmf/workflows/nightly/badge.svg)

The OpenRMF platform for multi-fleet robot management.

---

## Installation Instructions

First, please follow the installation instructions for ROS 2 Foxy,
[here is the binary install page for ROS 2 Foxy on Ubuntu 20.04](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html) or follow the installation instructions for ROS 2 Galactic,
[here is the binary install page for ROS 2 Galactic on Ubuntu 20.04](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html).

## Setup Gazebo repositories

Setup your computer to accept Gazebo packages from packages.osrfoundation.org.

```bash
sudo apt update
sudo apt install -y wget
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

### RMF Demos

A good way to install the `rmf` set of packages in one go is to install the one of the main [RMF Demos](https://github.com/open-rmf/rmf_demos) packages. This will pull all the rest of the OpenRMF packages as a dependency. The core of RMF demos is contained on the `rmf_demos` package. However, if you want to install it with simulation support, you should install the `rmf_demos_gz` or `rmf_demos_ign` package which come with gazebo or ignition support respectively. As an example, to install the ROS 2 release with gazebo support package, you would run:

For Foxy:-

```bash
sudo apt install ros-foxy-rmf-demos-gz
```

For Galactic:-

```bash
sudo apt install ros-galactic-rmf-demos-gz
```

## Building from sources

If you want to get the latest developments you might want to install from sources and compile OpenRMF yourself.

### Additional Dependencies

Install all non-ROS dependencies of OpenRMF packages,

```bash
sudo apt update && sudo apt install \
  git cmake python3-vcstool curl \
  qt5-default \
  -y
python3 -m pip install flask-socketio
sudo apt-get install python3-colcon*
```

### Install rosdep

`rosdep` helps install dependencies for ROS packages across various distros. It can be installed with:

```bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

### Download the source code

Setup a new ROS 2 workspace and pull in the demo repositories using `vcs`,

```bash
mkdir -p ~/rmf_ws/src
cd ~/rmf_ws
wget https://raw.githubusercontent.com/open-rmf/rmf/main/rmf.repos
vcs import src < rmf.repos
```

Ensure all ROS 2 prerequisites are fulfilled,

For Foxy:-

```bash
cd ~/rmf_ws
rosdep install --from-paths src --ignore-src --rosdistro foxy -y
```

For Galactic:-

```bash
cd ~/rmf_ws
rosdep install --from-paths src --ignore-src --rosdistro galactic -y
```

### Compiling Instructions

For Foxy:-

```bash
cd ~/rmf_ws
source /opt/ros/foxy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

For Galactic:-

```bash
cd ~/rmf_ws
source /opt/ros/galactic/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

> NOTE: The first time the build occurs, many simulation models will be downloaded from Ignition Fuel to populate the scene when the simulation is run.
> As a result, the first build can take a very long time depending on the server load and your Internet connection (for example, 60 minutes).

## Run RMF Demos

Demonstrations of OpenRMF are shown in [rmf_demos](https://github.com/open-rmf/rmf_demos/).

### Docker Containers

Alternatively, you can run RMF Demos by using [docker](https://docs.docker.com/engine/install/ubuntu/).

Pull docker image from `open-rmf/rmf` github registry (setup refer [here](https://docs.github.com/en/free-pro-team@latest/packages/using-github-packages-with-your-projects-ecosystem/configuring-docker-for-use-with-github-packages#authenticating-with-a-personal-access-token)).

```bash
docker pull ghcr.io/open-rmf/rmf/rmf_demos:latest
docker tag ghcr.io/open-rmf/rmf/rmf_demos:latest rmf:latest
```

Run it!

```bash

docker run -it --network host rmf:latest bash -c "export ROS_DOMAIN_ID=9; ros2 launch rmf_demos_gz office.launch.xml headless:=1"
```

This will run `rmf_demos` in headless mode. Open [this link](https://open-rmf.github.io/rmf-panel-js/) with a browser to start a task.

(Experimental) User can also run `rmf_demos` in “non-headless” graphical form, via [rocker](https://github.com/osrf/rocker).

| Packages                           | Version |
| ---------------------------------- | :-----: |
| ament_cmake_catch2                 |  1.2.0  |
| free_fleet                         |         |
| free_fleet_ros1                    |         |
| free_fleet_ros2                    |         |
| menge_vendor                       |  1.0.0  |
| rmf_battery                        |  0.1.3  |
| rmf_building_map_msgs              |  1.2.0  |
| rmf_cmake_uncrustify               |  1.2.0  |
| rmf_demos                          |         |
| rmf_demos_ign                      |         |
| rmf_demos_gz                       |         |
| rmf_demos_dashboard_resources      |         |
| rmf_demos_assets                   |         |
| rmf_demos_maps                     |         |
| rmf_demos_panel                    |         |
| rmf_demos_tasks                    |         |
| rmf_dispenser_msgs                 |  2.0.0  |
| rmf_door_msgs                      |  2.0.0  |
| rmf_fleet_msgs                     |  2.0.0  |
| rmf_ingestor_msgs                  |  2.0.0  |
| rmf_lift_msgs                      |  2.0.0  |
| rmf_task_msgs                      |  2.0.0  |
| rmf_traffic_msgs                   |  2.0.0  |
| rmf_workcell_msgs                  |  2.0.0  |
| rmf_charger_msgs                   |  2.0.0  |
| rmf_fleet_adapter                  |         |
| rmf_task_ros2                      |         |
| rmf_traffic_ros2                   |         |
| rmf_fleet_adapter_python           |         |
| rmf_robot_sim_gazebo_plugins       |         |
| rmf_robot_sim_ignition_plugins     |         |
| rmf_robot_sim_common               |         |
| rmf_building_sim_gazebo_plugins    |         |
| rmf_building_sim_ignition_plugins  |         |
| rmf_building_sim_common            |         |
| rmf_task                           |  2.0.0  |
| rmf_task_sequence                  |  2.0.0  |
| rmf_traffic                        |  2.0.0  |
| rmf_building_map_tools             |  1.5.1  |
| rmf_traffic_editor_test_maps       |  1.5.1  |
| rmf_traffic_editor                 |  1.5.1  |
| rmf_traffic_editor_assets          |  1.5.1  |
| rmf_utils                          |  1.4.0  |
| rmf_visualization_building_systems |         |
| rmf_visualization_fleet_states     |         |
| rmf_visualization_schedule         |         |
| rmf_visualization_rviz2_plugins    |         |
| rmf_visualization                  |         |
| rmf_visualization_msgs             |  1.2.0  |
| stubborn_buddies                   |  1.0.0  |
| stubborn_buddies_msgs              |  1.0.0  |
| rmf_api_msgs                       |  0.0.1  |

## Roadmap

A near-term roadmap of the entire OpenRMF project (including and beyond `rmf_traffic`) can be found in the user manual [here](https://osrf.github.io/ros2multirobotbook/roadmap.html).

## Integrating with RMF

Instructions on how to integrate your system with OpenRMF can be found [here](https://osrf.github.io/ros2multirobotbook/integration.html).
