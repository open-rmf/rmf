# Robotics Middleware Framework (Open-RMF)

![](media/rmf_banner.png)

![](https://github.com/open-rmf/rmf/workflows/build/badge.svg)
![Nightly](https://github.com/open-rmf/rmf/workflows/nightly/badge.svg)

The Open-RMF platform for multi-fleet robot management.

---

## Installation Instructions

Open-RMF is a collection of packages, some of which have ROS 2 dependencies.
For convenience, we distribute and install Open-RMF along with ROS 2 and is currently supported for the following ROS 2 distributions:
* [Iron Irwini](https://docs.ros.org/en/iron/index.html) (`iron`)
* [Humble Hawksbill](https://docs.ros.org/en/humble/index.html) (`humble`)
* [Rolling Ridley](https://docs.ros.org/en/rolling/index.html) (`rolling`)

We primarily support Debian packages on `Ubuntu Linux - Jammy Jellyfish (22.04)` and select RPM packages for `RHEL/Fedora` for both `amd64` and `aarch64` architectures.

Options for installing Open-RMF:
* [Binary packages (recommended)](#binary-packages)
* [Building from source](#building-from-source)

If you want to try Open-RMF we recommend installing the binaries. Building from source is better suited for developers who wish to add new features or fix bugs.

### Setup
Instruction below are aimed at `Ubuntu 22.04`.

First please follow the installation instructions to install the ROS 2 debian packages for the `distro` of choice from supported versions listed above.

Next, setup your computer to accept `Gazebo` packages from `packages.osrfoundation.org`.

```bash
sudo apt update
sudo apt install -y wget
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

### Binary packages
First follow the [setup instructions](#setup).
Finally install Open-RMF debian packages for the `distro` of choice.

```bash
sudo apt update && sudo apt install ros-<distro>-rmf-dev
```

This will install all necessary debian packages to run Open-RMF.
To try out some demonstrations, follow [these instructions](#rmf-demos).

### Building from source
First follow the [setup instructions](#setup).
Before proceeding, ensure that you not have any Open-RMF binaries installed on your system.
```bash
# optional
sudo apt purge ros-<distro>-rmf* && sudo apt autoremove
```

Install `rosdep` which helps to install dependencies for ROS packages across various distros

```bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```


```bash
sudo apt update && sudo apt install \
  git cmake python3-vcstool python3-pip curl \
  -y
python3 -m pip install flask-socketio fastapi uvicorn datamodel_code_generator
sudo apt-get install python3-colcon*
```

In order to get the latest developments we recommend you to install from sources and compile Open-RMF yourself.

#### Additional Dependencies

Install all non-ROS dependencies of Open-RMF packages,

```bash
sudo apt update && sudo apt install \
  git cmake python3-vcstool python3-pip curl \
  -y
python3 -m pip install flask-socketio fastapi uvicorn datamodel_code_generator
sudo apt-get install python3-colcon*
```

#### Install rosdep

`rosdep` helps install dependencies for ROS packages across various distros. It can be installed with:

```bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

#### Download the source code

> Note all repositories now have a `galactic-devel` branch. New changes will be targeted for `ROS2 Humble`. If any change is compatible with `ROS2 galactic` they will be backported to the `galactic-devel` branch.

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
for `humble`

```bash
cd ~/rmf_ws
rosdep install --from-paths src --ignore-src --rosdistro humble -y
```

#### Compiling Instructions

> NOTE: Due to newer changes in the source build, there might be conflicts and compilation errors with older header files installed by the binaries. Please remove the binary installations before building from source, using `sudo apt remove ros-humble-rmf*`.

Compiling on `Ubuntu 22.04`:

##### Install clang

```bash
sudo apt update
sudo apt install clang clang-tools lldb lld libstdc++-12-dev
```

**NOTE: We strongly recommend compiling Open-RMF packages with `clang` as compiler and `lld` as linker.**

##### Compile using clang

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

### Binary install

Latest Open-RMF binary packages are available for Ubuntu Jammy 22.04 for the `Humble` and `Rolling` releases of ROS 2. Older releases are also available on Ubuntu Focal 20.04 for `Foxy` and `Galactic`. Most Open-RMF packages have the prefix `rmf` on their name, therefore, you can find them by searching for the pattern `ros-<ro2distro>-rmf`, e.g., for humble it would be:

```bash
apt-cache search ros-humble-rmf
```

## RMF Demos for binary installs

**Note:** RMF Demos package cannot be installed on Humble distro because of an underlying [issue](https://github.com/open-rmf/rmf_demos/issues/166) with the release of a bad version of fastapi in jammy. You can install the package from [source](https://github.com/open-rmf/rmf/discussions/267).
It is important to have `fastapi` installed via `pip` and not as an Ubuntu system package (ie, via `apt install`) for the reasons documented above.
Please follow the instructions in the [Additional Dependencies](#additional-dependencies) section to install `fastapi` along with other dependencies needed to run Open-RMF demos.

[//]: # (A good way to install the `rmf` set of packages in one go is to install the one of the main [RMF Demos]&#40;https://github.com/open-rmf/rmf_demos&#41; packages. This will pull all the rest of the Open-RMF packages as a dependency. The core of Open-RMF demos is contained on the `rmf_demos` package. However, if you want to install it with simulation support, you should install the `rmf_demos_gz` or `rmf_demos_gz_classic` package which come with Gazebo or Gazebo Classic support respectively. As an example, to install the ROS 2 Humble release with Gazebo support package, you would run:)

[//]: # ()
[//]: # (```bash)

[//]: # (sudo apt install ros-humble-rmf-demos-gz-classic)
[//]: # (```)

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

Demonstrations of Open-RMF are shown in [rmf_demos](https://github.com/open-rmf/rmf_demos/).

## Roadmap

A near-term roadmap of the entire Open-RMF project (including and beyond `rmf_traffic`) can be found in the user manual [here](https://osrf.github.io/ros2multirobotbook/roadmap.html).

## Integrating with RMF

Instructions on how to integrate your system with Open-RMF can be found [here](https://osrf.github.io/ros2multirobotbook/integration.html).

## Open sourced adapters

A number of commercial robots, infrastructure systems, workcells and devices have been integrated with Open-RMF and links to their adapters are available in the [awesome_adapter](https://github.com/open-rmf/awesome_adapters) repository

Help us add to this list!

A helpful starting point for integrating your fleet with RMF is the [fleet_adapter_template](https://github.com/open-rmf/free_fleet) package.
