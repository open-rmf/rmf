#-----------------------
# Stage 1 - Dependencies
#-----------------------

FROM ros:jazzy AS builder

ENV PIP_BREAK_SYSTEM_PACKAGES=1
RUN apt-get update && apt-get install -y ros-dev-tools

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys D2486D2DD83DB69272AFE98867170598AF249743

RUN mkdir $HOME/rmf_demos_ws
WORKDIR $HOME/rmf_demos_ws
RUN mkdir src
RUN rosdep update --rosdistro $ROS_DISTRO

# This replaces: wget https://raw.githubusercontent.com/open-rmf/rmf/main/rmf.repos
ENV DEBIAN_FRONTEND=noninteractive
COPY rmf.repos rmf.repos
RUN vcs import src < rmf.repos \
    && apt-get update \
    && apt-get upgrade -y \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -yr \
    && rm -rf /var/lib/apt/lists/*

#-----------------
# Stage 2 - build
#-----------------

# colcon compilation
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

#----------
# Stage 3
#----------

# TODO: rosdep doesn't support installing only exec dependencies (https://github.com/ros-infrastructure/rosdep/pull/727)
#   When the PR is merged, we can do a multi-stage build and include only whats needed at runtime.
# FROM ros:foxy
# COPY --from=0 /root/rmf/install /opt/rmf
# RUN rosdep ...
# COPY --from=builder /root/rmf/install /opt/rmf

# cleanup
RUN rm -rf build devel src \
  && sed -i '$isource "/rmf_demos_ws/install/setup.bash"' /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
