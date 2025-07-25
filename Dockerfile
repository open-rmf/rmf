ARG ROS_DISTRO=jazzy
ARG BASE_IMAGE=ros:${ROS_DISTRO}-ros-base
FROM $BASE_IMAGE
ARG REPOS_FILE=rmf.repos

RUN apt-get update && apt-get install -y ros-dev-tools ros-${ROS_DISTRO}-rmw-cyclonedds-cpp ros-${ROS_DISTRO}-rmw-zenoh-cpp

RUN mkdir -p /rmf_demos_ws/src
WORKDIR /rmf_demos_ws
RUN rosdep update --rosdistro ${ROS_DISTRO}

ENV DEBIAN_FRONTEND=noninteractive
COPY ${REPOS_FILE} rmf.repos
RUN vcs import src < rmf.repos \
    && apt-get update \
    && apt-get upgrade -y \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -yr \
    && rm -rf /var/lib/apt/lists/*

# colcon compilation
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# cleanup
RUN rm -rf build log src \
  && sed -i '$isource "/rmf_demos_ws/install/setup.bash"' /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
