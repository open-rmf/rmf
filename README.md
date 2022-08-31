# Robotics Middleware Framework (RMF)

![](media/rmf_banner.png)

![](https://github.com/open-rmf/rmf/workflows/build/badge.svg)
![Nightly](https://github.com/open-rmf/rmf/workflows/nightly/badge.svg)

The OpenRMF platform for multi-fleet robot management.

---

For specific rmf versions follow the instructions given below:

## Installation Instructions

These are the current Open-RMF binary releases available:

| RMF Version | Installation Instructions                                                        | Supported distros                                    | Supported ROS2 versions |
| ----------- | -------------------------------------------------------------------------------- | ---------------------------------------------------- | ----------------------- |
| 21.09       |  [Installation instructions](https://github.com/open-rmf/rmf/tree/release/21.09) | Ubuntu 20.04, Ubuntu 21.09, RHEL 8 (deployment only) | Foxy, Galactic          |
| 22.02       |  [Installation instructions](https://github.com/open-rmf/rmf/tree/release/22.02) | Ubuntu 22.04                                         | Humble                  |

## Building from sources

### Additional Dependencies

Install all non-ROS dependencies of OpenRMF packages,

```bash
sudo apt update && sudo apt install \
  git cmake python3-vcstool curl \
  qt5-default \
  -y
python3 -m pip install flask-socketio fastapi uvicorn
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

> Note all repositories now have a `galactic-devel` branch. New changes will be targeted for `humble` however if any change can be backported to `galactic` they will be added to the `galactic-devel` branch.

Setup a new ROS 2 workspace and pull in the demo repositories using `vcs`,

```bash
mkdir -p ~/rmf_ws/src
cd ~/rmf_ws
wget https://raw.githubusercontent.com/open-rmf/rmf/main/rmf.repos
vcs import src < rmf.repos
```

Ensure all ROS 2 prerequisites are fulfilled,

you can subsutitute your distro name for `<your ros distro>`

Example:
for `galactic`

```bash
cd ~/rmf_ws
rosdep install --from-paths src --ignore-src --rosdistro galactic -y
```

### Compiling Instructions

> NOTE: Due to newer changes in the source build, there might be conflicts and compilation errors with older header files installed by the binaries. Please remove the binary installations before building from source, using `sudo apt remove ros-galactic-rmf*`.

Compiling on `Ubuntu 20.04`:

```bash
cd ~/rmf_ws
source /opt/ros/<your ros distro>/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

> NOTE: The first time the build occurs, many simulation models will be downloaded from Ignition Fuel to populate the scene when the simulation is run.
> As a result, the first build can take a very long time depending on the server load and your Internet connection (for example, 60 minutes).

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

## Run RMF Demos

Demonstrations of OpenRMF are shown in [rmf_demos](https://github.com/open-rmf/rmf_demos/).

## Roadmap

A near-term roadmap of the entire OpenRMF project (including and beyond `rmf_traffic`) can be found in the user manual [here](https://osrf.github.io/ros2multirobotbook/roadmap.html).

## Integrating with RMF

Instructions on how to integrate your system with OpenRMF can be found [here](https://osrf.github.io/ros2multirobotbook/integration.html).

## Open sourced fleet adapters

A number of commercial robots have been integrated with RMF and links to their adapters are available below.

- [Gaussian Ecobots](https://github.com/open-rmf/fleet_adapter_ecobot)
- [OTTO Motors](https://github.com/open-rmf/fleet_adapter_clearpath) (and robots running the Clearpath Autonomy stack)
- [Mobile Industrial Robots: MiR](https://github.com/osrf/fleet_adapter_mir)
- [Temi- the personal robot](https://github.com/open-rmf/temi_fleet_adapter_python)

Help us add to this list!

A helpful starting point for integrating your fleet with RMF is the [fleet_adapter_template](https://github.com/open-rmf/free_fleet) package.
