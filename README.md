# Robotics Middleware Framework (RMF)

The OpenRMF platform for multi-fleet robot management.

---

## Ubuntu Installation Instructions

| Ubuntu version | ROS2 version     | Installation Instructions                              | Latest version                                              |
| -------------- | ---------------- | ------------------------------------------------------ | ----------------------------------------------------------- |
| 20.04 LTS      | Foxy             | [ubuntu20.04_foxy](./docs/ubuntu20.04_foxy.md)         | [21.09](https://github.com/open-rmf/rmf/releases/tag/21.09) |
|                | Galactic         | [ubuntu20.04_galactic](./docs/ubuntu20.04_galactic.md) | [21.09](https://github.com/open-rmf/rmf/releases/tag/21.09) |
| 22.04 LTS      | Humble Hawksbill | [ubuntu22.04_humble](./docs/ubuntu22.04_humble.md)     | 22.04 Under developement |                      |

## Run RMF Demos

Demonstrations of OpenRMF are shown in [rmf_demos](https://github.com/open-rmf/rmf_demos/).

### Docker Containers

Alternatively, you can run RMF Demos by using [docker](https://docs.docker.com/engine/install/ubuntu/).

Pull docker image from `open-rmf/rmf` github registry (setup refer [[https://docs.github.com/en/free-pro-team@latest/packages/using-github-packages-with-your-projects-ecosystem/configuring-docker-for-use-with-github-packages#authenticating-with-a-personal-access-token]]).

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
