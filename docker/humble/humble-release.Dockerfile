ARG BUILDER_NS="open-rmf/rmf"
ARG TAG="latest"
ARG BASE_REGISTRY="docker.io"

FROM $BUILDER_NS/rmf-deps-humble:$TAG

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

# colcon compilation
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# cleanup
RUN rm -rf build devel src \
  && sed -i '$isource "/rmf_demos_ws/install/setup.bash"' /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
