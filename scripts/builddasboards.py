import yaml

# load build_combinations.yaml file
with open("build_combinations.yaml", "r") as f:
    build_combinations = yaml.load(f, Loader=yaml.FullLoader)

ros_versions = build_combinations["build_types"]



## TODO: Try to get from colcon list
packages = [
    "ament_cmake_catch2",
    "menge_vendor",
    "nlohmann_json_schema_validator_vendor",
    "pybind11_json_vendor",
    "rmf_api_msgs",
    "rmf_battery",
    "rmf_building_map_msgs",
    "rmf_building_map_tools",
    "rmf_building_sim_common",
    "rmf_building_sim_gz_classic_plugins",
    "rmf_building_sim_gz_plugins",
    "rmf_charger_msgs",
    "rmf_demos",
    "rmf_demos_assets",
    "rmf_demos_bridges",
    "rmf_demos_dashboard_resources",
    "rmf_demos_fleet_adapter",
    "rmf_demos_gz",
    "rmf_demos_gz_classic",
    "rmf_demos_maps",
    "rmf_demos_panel",
    "rmf_demos_tasks",
    "rmf_dispenser_msgs",
    "rmf_door_msgs",
    "rmf_fleet_adapter",
    "rmf_fleet_adapter_python",
    "rmf_fleet_msgs",
    "rmf_ingestor_msgs",
    "rmf_lift_msgs",
    "rmf_obstacle_msgs",
    "rmf_robot_sim_common",
    "rmf_robot_sim_gz_classic_plugins",
    "rmf_robot_sim_gz_plugins",
    "rmf_scheduler_msgs",
    "rmf_site_map_msgs",
    "rmf_task",
    "rmf_task_msgs",
    "rmf_task_ros2",
    "rmf_task_sequence",
    "rmf_traffic",
    "rmf_traffic_editor",
    "rmf_traffic_editor_assets",
    "rmf_traffic_editor_test_maps",
    "rmf_traffic_examples",
    "rmf_traffic_msgs",
    "rmf_traffic_ros2",
    "rmf_utils",
    "rmf_visualization",
    "rmf_visualization_building_systems",
    "rmf_visualization_fleet_states",
    "rmf_visualization_floorplans",
    "rmf_visualization_msgs",
    "rmf_visualization_navgraphs",
    "rmf_visualization_obstacles",
    "rmf_visualization_rviz2_plugins",
    "rmf_visualization_schedule",
    "rmf_websocket",
    "rmf_workcell_msgs",
]

jenkins_base_url = "https://build.ros2.org"
jenkins_job_url_template = jenkins_base_url + "/job/{ros_version}{package_type}_{first}_{package_name}_{last}"
icon_url_template = jenkins_job_url_template + "/badge/icon"

# | ament_cmake_catch2                    |                              [![ament_cmake_catch2](https://build.ros2.org/job/Rdev__ament_cmake_catch2__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Rdev__ament_cmake_catch2__ubuntu_jammy_amd64/)                              |                                                          [![ament_cmake_catch2](https://build.ros2.org/job/Rbin_uJ64__ament_cmake_catch2__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uJ64__ament_cmake_catch2__ubuntu_jammy_amd64__binary/)
md_status_template = "[![{package_name}]({icon_url})]({jenkins_url})"
table_row_string = "| {package_name} | {statuses_str} |  "

def generate_md_body():
    body = ""
    for ros_version, package_types in ros_versions.items():
        body += f"## {ros_version}\n"
        if package_types is None:
            continue
        body += generate_table(ros_version)+"\n\n"
    return body

def generate_table_header(ros_version):
    header = "| Package | "
    combos = []
    for package_type, combs in ros_versions[ros_version].items():
        if combs is None:
            combs = [("", "")]
        for (first, last) in combs:
            combos.append(f"{ros_version}{package_type} {last}")
    header += " | ".join(combos) + " |\n"
    header += "|:---:| " + " | ".join([":---:"for _ in combos]) + " |"
    return header

def generate_table(ros_version):
    table = generate_table_header(ros_version)+"\n"
    table += generate_table_body(ros_version)
    return table


def generate_table_body(ros_version):
    table = ""
    for package_name in packages:
        table += generate_line(package_name, ros_version)
    return table


def generate_line(package_name, ros_version):
    statuses = []
    for package_type, combs in ros_versions[ros_version].items():
        if combs is None:
            combs = [("", "")]
        for (first, last) in combs:
            new_package_name = f"_{package_name}" if first else package_name
            new_package_name = f"{new_package_name}_" if last else new_package_name
            icon_url = icon_url_template.format(ros_version=ros_version, package_type=package_type, first=first, package_name=new_package_name, last=last)
            jenkins_url = jenkins_job_url_template.format(ros_version=ros_version, package_type=package_type, first=first, package_name=new_package_name, last=last)
            statuses.append(md_status_template.format(package_name=package_name, icon_url=icon_url, jenkins_url=jenkins_url))
    return table_row_string.format(package_name=package_name, statuses_str=" | ".join(statuses))+"\n"

if __name__ == '__main__':
    # write header and body to builds.md file
    with open("../reports/builds.md", "w") as f:
        f.writelines(generate_md_body()



 rmf_robot_sim_common/CHANGELOG.rst rmf_building_sim_gz_plugins/CHANGELOG.rst rmf_building_sim_common/CHANGELOG.rst rmf_building_sim_gz_classic_plugins/CHANGELOG.rst rmf_robot_sim_gz_classic_plugins/CHANGELOG.rst rmf_robot_sim_gz_plugins/CHANGELOG.rst

