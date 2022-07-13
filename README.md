# Robotics Middleware Framework (RMF)

![](media/rmf_banner.png)

![](https://github.com/open-rmf/rmf/workflows/build/badge.svg)
![Nightly](https://github.com/open-rmf/rmf/workflows/nightly/badge.svg)

The OpenRMF platform for multi-fleet robot management.

---
## Install ROS 2 Humble

First, please follow the installation instructions for ROS 2 Humble.
If you are on an Ubuntu 22.04 LTS machine (as recommended), [here is the binary install page for ROS 2 Humble on Ubuntu 22.04](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

## Setup Gazebo repositories

Setup your computer to accept Gazebo packages from packages.osrfoundation.org.

```bash
sudo apt update
sudo apt install -y wget
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

## Binary install

Latest OpenRMF binary packages are available for Ubuntu Jammy 22.04 for the `Humble` and `Rolling` releases of ROS 2. Older releases are also available on Ubuntu Focal 20.04 for `Foxy` and `Galactic`. Most OpenRMF packages have the prefix `rmf` on their name, therefore, you can find them by them by searching for the pattern `ros-<ro2distro>-rmf`, e.g., for galatic it would be:

```bash
apt-cache search ros-humble-rmf
```

### RMF Demos

A good way to install the `rmf` set of packages in one go is to install the one of the main [RMF Demos](https://github.com/open-rmf/rmf_demos) packages. This will pull all the rest of the OpenRMF packages as a dependency. The core of RMF demos is contained on the `rmf_demos` package. However, if you want to install it with simulation support, you should install the `rmf_demos_gz` or `rmf_demos_ign` package which come with gazebo or ignition support respectively. As an example, to install the ROS 2 Humble release with gazebo support package, you would run:

```bash
sudo apt install ros-humble-rmf-demos-gz
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
wget https://raw.githubusercontent.com/open-rmf/rmf/main/rmf-simulation.repos
vcs import src < rmf-simulation.repos
```

Ensure all ROS 2 prerequisites are fulfilled,
```
cd ~/rmf_ws
rosdep install --from-paths src --ignore-src --rosdistro humble -y
```

### Compiling Instructions

> NOTE: Due to newer changes in the source build, there might be conflicts and compilation errors with older header files installed by the binaries. Please remove the binary installations before building from source, using `sudo apt remove ros-humble-rmf*`.

Compiling on `Ubuntu 22.04`:

Install clang

```bash
sudo apt update
sudo apt install clang lldb lld
```

**NOTE: RMF does not support building on gcc.**

Compile using clang

```bash
cd ~/rmf_ws
source /opt/ros/humble/setup.bash
CXX=clang++ LDFLAGS='-fuse-ld=lld' colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

> NOTE: The first time the build occurs, many simulation models will be downloaded from Ignition Fuel to populate the scene when the simulation is run.
As a result, the first build can take a very long time depending on the server load and your Internet connection (for example, 60 minutes).

## Run RMF Demos

Demonstrations of OpenRMF are shown in [rmf_demos](https://github.com/open-rmf/rmf_demos/).

### Docker Containers
Alternatively, you can run RMF Demos by using [docker](https://docs.docker.com/engine/install/ubuntu/).

```bash
docker run -it --network host ghcr.io/open-rmf/rmf/rmf_demos:latest bash -c "export ROS_DOMAIN_ID=9; ros2 launch rmf_demos_gz office.launch.xml headless:=1"
```
This will run `rmf_demos` in headless mode. Open [this link](https://open-rmf.github.io/rmf-panel-js/) with a browser to start a task.

(Experimental) User can also run `rmf_demos` in “non-headless” graphical form, via [rocker](https://github.com/osrf/rocker).

## Roadmap

A near-term roadmap of the entire OpenRMF project (including and beyond `rmf_traffic`) can be found in the user manual [here](https://osrf.github.io/ros2multirobotbook/roadmap.html).

## Integrating with RMF

Instructions on how to integrate your system with OpenRMF can be found [here](https://osrf.github.io/ros2multirobotbook/integration.html).

## Open sourced fleet adapters

A number of commercial robots have been integrated with RMF and links to their adapters are available below.

* [Gaussian Ecobots](https://github.com/open-rmf/fleet_adapter_ecobot)
* [OTTO Motors](https://github.com/open-rmf/fleet_adapter_clearpath) (and robots running the Clearpath Autonomy stack)
* [Mobile Industrial Robots: MiR](https://github.com/osrf/fleet_adapter_mir)
* [Temi- the personal robot](https://github.com/open-rmf/temi_fleet_adapter_python)

Help us add to this list!

A helpful starting point for integrating your fleet with RMF is the [fleet_adapter_template](https://github.com/open-rmf/free_fleet) package.
