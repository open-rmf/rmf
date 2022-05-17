First, please follow the installation instructions for ROS 2 Galactic,
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

A good way to install the `rmf` set of packages in one go is to install the one of the main [RMF Demos](https://github.com/open-rmf/rmf_demos) packages. This will pull all the rest of the OpenRMF packages as a dependency. The core of RMF demos is contained on the `rmf_demos` package. However, if you want to install it with simulation support, you should install the `rmf_demos_gz` or `rmf_demos_ign` package which come with gazebo or ignition support respectively. As an example, to install the ROS 2 Galactic release with gazebo support package, you would run:

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
```bash
cd ~/rmf_ws
rosdep install --from-paths src --ignore-src --rosdistro galactic -y
```

### Compiling Instructions

```bash
cd ~/rmf_ws
source /opt/ros/galactic/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

> NOTE: The first time the build occurs, many simulation models will be downloaded from Ignition Fuel to populate the scene when the simulation is run.
As a result, the first build can take a very long time depending on the server load and your Internet connection (for example, 60 minutes).

