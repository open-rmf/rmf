# Open-RMF 22.09

![](https://github.com/open-rmf/rmf/workflows/build/badge.svg)

![Nightly](https://github.com/open-rmf/rmf/workflows/nightly/badge.svg)

The Open-RMF platform for multi-fleet robot management.

---

## Installation Instructions

First, please follow the installation instructions for ROS 2 Humble,
[here is the binary install page for ROS 2 Humble on Ubuntu 22.04](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

## Setup Gazebo repositories

Setup your computer to accept Gazebo packages from packages.osrfoundation.org.

```bash
sudo apt update
sudo apt install -y wget
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

## Binary installation

> **Note:** RMF Demos package cannot be installed on Humble distro because of an underlying [issue](https://github.com/open-rmf/rmf_demos/issues/166) with the release of a bad version of fastapi in jammy. You can install the package from [source](https://github.com/open-rmf/rmf/discussions/267).
It is important to have `fastapi` installed via `pip` and not as an Ubuntu system package (ie, via `apt install`) for the reasons documented above.
Please follow the instructions in the [Additional Dependencies](#additional-dependencies) section to install `fastapi` along with other dependencies needed to run Open-RMF demos.

All software required for Open-RMF is available as packages in the official ROS2 Humble repository.

You can search for all available packages with 
sudo apt search ros-humble-rmf

Installing this package will install all the necessary dependencies:
```shell
sudo apt install ros-humble-rmf-visualization 
```

If you use `Gazebo classic` for simulations you will also need to install
```shell
sudo apt install ros-humble-rmf-robot-sim-gz-classic-plugins ros-humble-rmf-building-sim-gz-classic-plugins
```

On the other hand, if you use `Gazebo` you must install
```shell
sudo apt install ros-humble-rmf-robot-sim-gz-plugins ros-humble-rmf-building-sim-gz-plugins
```

To check your installation you can run:
```shell
ros2 launch rmf_visualization visualization.launch.xml
```

## Building from sources

If you prefer to compile Open-RMF yourself you can follow these steps.

### Additional Dependencies

Install all non-ROS dependencies of Open-RMF packages,

```bash
sudo apt update && sudo apt install \
  git cmake python3-vcstool curl \
  -y
python3 -m pip install flask-socketio fastapi uvicorn datamodel_code_generator
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

Setup a new ROS 2 workspace and pull in the repositories using `vcs`,

```bash
mkdir -p ~/rmf_ws/src
cd ~/rmf_ws
wget https://raw.githubusercontent.com/open-rmf/rmf/release/22.09/rmf.repos
vcs import src < rmf.repos
```

Ensure all ROS 2 prerequisites are fulfilled, for `humble`

```bash
cd ~/rmf_ws
rosdep install --from-paths src --ignore-src --rosdistro humble -y
```

### Compiling Instructions

> NOTE: Due to newer changes in the source build, there might be conflicts and compilation errors with older header files installed by the binaries. Please remove the binary installations before building from source, using `sudo apt remove ros-humble-rmf*`.


#### Install clang

**NOTE: We strongly recommend compiling Open-RMF packages with `clang` as compiler and `lld` as linker.**
```bash
sudo apt update
sudo apt install clang lldb lld
```


#### Compile using clang

Update colcon mixin which is a one time step:

```bash
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default
```

Compile the workspace:

```bash
cd ~/rmf_ws
source /opt/ros/humble/setup.bash

export CXX=clang++
export CC=clang
colcon build --mixin release lld
```

> NOTE: The first time the build occurs, many simulation models will be downloaded from Ignition Fuel to populate the scene when the simulation is run.
> As a result, the first build can take a very long time depending on the server load and your Internet connection (for example, 60 minutes).


## Run RMF Demos

Demonstrations of Open-RMF are shown in [rmf_demos](https://github.com/open-rmf/rmf_demos/).

### Docker Containers

Alternatively, you can run Open-RMF Demos by using [docker](https://docs.docker.com/engine/install/ubuntu/).

Pull docker image from `open-rmf/rmf` github registry (setup refer [here](https://docs.github.com/en/free-pro-team@latest/packages/using-github-packages-with-your-projects-ecosystem/configuring-docker-for-use-with-github-packages#authenticating-with-a-personal-access-token)).

```bash
docker pull ghcr.io/open-rmf/rmf/rmf_demos:22_09
docker tag ghcr.io/open-rmf/rmf/rmf_demos:22_09 rmf:latest
```

Run it!

```bash

docker run -it --network host rmf:latest bash -c "export ROS_DOMAIN_ID=9; ros2 launch rmf_demos_gz office.launch.xml headless:=1"
```
This will run `rmf_demos` in headless mode. Open [this link](https://open-rmf.github.io/rmf-panel-js/) with a browser to start a task.

(Experimental) User can also run `rmf_demos` in “non-headless” graphical form, via [rocker](https://github.com/osrf/rocker).


## Package Information

| Packages                                         | Github link                                                                   | Version |
|--------------------------------------------------|-------------------------------------------------------------------------------|---------|
| rmf/ament_cmake_catch2                           | https://github.com/open-rmf/ament_cmake_catch2/tree/humble                    | humble  |
| rmf/rmf_api_msgs                                 | https://github.com/open-rmf/rmf_api_msgs/tree/humble                          | humble  |
| rmf/rmf_battery                                  | https://github.com/open-rmf/rmf_battery/tree/humble                           | humble  |
| rmf/rmf_building_map_msgs                        | https://github.com/open-rmf/rmf_building_map_msgs/tree/humble                 | humble  |
| rmf/rmf_internal_msgs                            | https://github.com/open-rmf/rmf_internal_msgs/tree/humble                     | humble  |
| rmf/rmf_ros2                                     | https://github.com/open-rmf/rmf_ros2/tree/humble                              | humble  |
| rmf/rmf_simulation                               | https://github.com/open-rmf/rmf_simulation/tree/humble                        | humble  |
| rmf/rmf_task                                     | https://github.com/open-rmf/rmf_task/tree/humble                              | humble  |
| rmf/rmf_traffic                                  | https://github.com/open-rmf/rmf_traffic/tree/humble                           | humble  |
| rmf/rmf_traffic_editor                           | https://github.com/open-rmf/rmf_traffic_editor/tree/humble                    | humble  |
| rmf/rmf_utils                                    | https://github.com/open-rmf/rmf_utils/tree/humble                             | humble  |
| rmf/rmf_visualization                            | https://github.com/open-rmf/rmf_visualization/tree/humble                     | humble  |
| rmf/rmf_visualization_msgs                       | https://github.com/open-rmf/rmf_visualization_msgs/tree/humble                | humble  |
| demonstrations/rmf_demos                         | https://github.com/open-rmf/rmf_demos/tree/humble                             | humble  |
| thirdparty/menge_vendor                          | https://github.com/open-rmf/menge_vendor/tree/humble                          | humble  |
| thirdparty/nlohmann_json_schema_validator_vendor | https://github.com/open-rmf/nlohmann_json_schema_validator_vendor/tree/humble | humble  |
| thirdparty/pybind11_json_vendor                  | https://github.com/open-rmf/pybind11_json_vendor/tree/humble                  | humble  |

