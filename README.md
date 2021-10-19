# RMF

![](https://github.com/open-rmf/rmf/workflows/build/badge.svg)

![Nightly](https://github.com/open-rmf/rmf/workflows/nightly/badge.svg)

The OpenRMF platform for multi-fleet robot management.

---
## Install ROS 2 Galactic

First, please follow the installation instructions for ROS 2 Galactic.
If you are on an Ubuntu 20.04 LTS machine (as recommended), [here is the binary install page for ROS 2 Galactic on Ubuntu 20.04](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html).

## Setup Gazebo repositories

Setup your computer to accept Gazebo packages from packages.osrfoundation.org.

```bash
sudo apt update
sudo apt install -y wget
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

## Binary install

OpenRMF binary packages are available for Ubuntu Focal 20.04 for the `Foxy`, `Galactic` and `Rolling` releases of ROS 2. Most OpenRMF packages have the prefix `rmf` on their name, therefore, you can find them by them by searching for the pattern `ros-<ro2distro>-rmf`, e.g., for galatic it would be:

```bash
apt-cache search ros-galactic-rmf
```

A good way to install the `rmf` set of packages in one go is to install the `rmf_demos` package. This will pull all the rest of the OpenRMF packages as a dependency. As an example, for Galactic, you would run:

```bash
sudo apt install ros-galactic-rmf-demos
```

## Building from sources

If you want to get the latest developments you might want to install from sources and compile OpenRMF yourself.


### Additional Dependencies

Install all non-ROS dependencies of RMF packages,

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
```
cd ~/rmf_ws
rosdep install --from-paths src --ignore-src --rosdistro galactic -yr
```

### Compiling Instructions

On `Ubuntu 20.04`:

```bash
cd ~/rmf_ws
source /opt/ros/galactic/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

> NOTE: The first time the build occurs, many simulation models will be downloaded from Ignition Fuel to populate the scene when the simulation is run.
As a result, the first build can take a very long time depending on the server load and your Internet connection (for example, 60 minutes).

## Run RMF Demos

Demonstrations of RMF is shown in [rmf_demos](https://github.com/open-rmf/rmf_demos/).

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

## Roadmap

A near-term roadmap of the entire RMF project (including and beyond `rmf_traffic`) can be found in the user manual [here](https://osrf.github.io/ros2multirobotbook/roadmap.html).

## Integrating with RMF

Instructions on how to integrate your system with RMF can be found [here](https://osrf.github.io/ros2multirobotbook/integration.html).
