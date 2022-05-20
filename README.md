# RMF

![](https://github.com/open-rmf/rmf/workflows/build/badge.svg)

![Nightly](https://github.com/open-rmf/rmf/workflows/nightly/badge.svg)

The OpenRMF platform for multi-fleet robot management.

---

## Installation Instructions

| ROS2 version | Installation Instructions                  |
| ------------ | ------------------------------------------ |
| Foxy         | [foxy](./docs/foxy.md)         |
| Galactic     | [galactic](./docs/galactic.md) |

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

## Roadmap

A near-term roadmap of the entire OpenRMF project (including and beyond `rmf_traffic`) can be found in the user manual [here](https://osrf.github.io/ros2multirobotbook/roadmap.html).

## Integrating with RMF

Instructions on how to integrate your system with OpenRMF can be found [here](https://osrf.github.io/ros2multirobotbook/integration.html).
