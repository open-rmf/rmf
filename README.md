# RMF

![](https://github.com/open-rmf/rmf/workflows/build/badge.svg)

The OpenRMF platform for multi-fleet robot management.

---
## Install ROS 2 Foxy

First, please follow the installation instructions for ROS 2 Foxy.
If you are on an Ubuntu 20.04 LTS machine (as recommended), [here is the binary install page for ROS 2 Foxy on Ubuntu 20.04](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/)

## Install rosdep
`rosdep` helps install dependencies for ROS packages across various distros. It can be installed with:
```bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

## Additional Dependencies

Setup your computer to accept Gazebo packages from packages.osrfoundation.org.

```bash
sudo apt update
sudo apt install -y wget
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```
Install all non-ROS dependencies of RMF packages,

```bash
sudo apt update && sudo apt install \
  git cmake python3-vcstool curl \
  qt5-default \
  ignition-edifice \
  -y
python3 -m pip install flask-socketio
sudo apt-get install python3-colcon*
```

## Download the source code
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
rosdep install --from-paths src --ignore-src --rosdistro foxy -yr
```

## Compiling Instructions

On `Ubuntu 20.04`:

```bash
cd ~/rmf_ws
source /opt/ros/foxy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

> NOTE: The first time the build occurs, many simulation models will be downloaded from Ignition Fuel to populate the scene when the simulation is run.
As a result, the first build can take a very long time depending on the server load and your Internet connection (for example, 60 minutes).

> The models required for each of the demo worlds will be automatically downloaded into `~/.gazebo/models` from Ignition Fuel when building the package `rmf_demo_maps`. If you notice something wrong with the models in the simulation, your `~/.gazebo/models` path might contain deprecated models not from Fuel. An easy way to solve this is to remove all models except for `sun` and `ground_plane` from `~/.gazebo/model`s, and perform a clean rebuild of the package `rmf_demo_maps`.

## Run RMF Demos

Demonstrations of RMF is shown in [rmf_demos](https://github.com/open-rmf/rmf_demos/)

### Docker Containers
Alternatively, you can run RMF Demos by using [docker](https://docs.docker.com/engine/install/ubuntu/).

Pull docker image from `opern-rm/rmf` github registry (setup refer [here](https://docs.github.com/en/free-pro-team@latest/packages/using-github-packages-with-your-projects-ecosystem/configuring-docker-for-use-with-github-packages#authenticating-with-a-personal-access-token)).
```bash
docker pull docker.pkg.github.com/open-rmf/rmf/rmf_demos:latest
docker tag docker.pkg.github.com/open-rmf/rmf/rmf_demos:latest rmf:latest
```

Run it!
```bash
docker run -it --network host rmf:latest bash -c "export ROS_DOMAIN_ID=9; ros2 launch rmf_demos office.launch.xml headless:=1"
```
This will run `rmf_demos` in headless mode. Open `localhost:5000` with a browser to start a task.

(Experimental) User can also run `rmf_demos` in “non-headless” graphical form, via [rocker](https://github.com/osrf/rocker).

## Roadmap

A near-term roadmap of the entire RMF project (including and beyond `rmf_core`) can be found in the user manual [here](https://osrf.github.io/ros2multirobotbook/roadmap.html).

## Integrating with RMF

Instructions on how to integrate your system with RMF can be found [here](https://osrf.github.io/ros2multirobotbook/integration.html).