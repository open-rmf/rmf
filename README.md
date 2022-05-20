# Robotics Middleware Framework (RMF)

The OpenRMF platform for multi-fleet robot management.

---

## Installation Instructions

| RMF Version | Installation Instructions                                                        | Supported distros                                   | Supported ROS2 versions |
| ----------- | -------------------------------------------------------------------------------- | --------------------------------------------------- | ----------------------- |
| 21.09       |  [installation instructions](https://github.com/open-rmf/rmf/tree/release/21.09) | Ubuntu 20.04, Ubuntu 21.09, RHEL 8(deployment only) | Foxy,Galactic           |
| 22.04       |                                                                                  | Ubuntu 22.04                                        | Humble Hawksbill        |

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
