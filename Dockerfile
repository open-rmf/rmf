#-----------------------
# Stage 1 - Dependencies
#-----------------------
ARG ROS_DISTRO=humble
FROM ros:$ROS_DISTRO AS builder

# qt5-default become obsolete since 21.04 https://bugs.launchpad.net/ubuntu/+source/qtbase-opensource-src/+bug/1926802
RUN apt-get update \
  && apt-get install -y \
    cmake \
    curl \
    git \
    python3-colcon-common-extensions \
    python3-vcstool \
    qtbase5-dev \
    qt5-qmake \
    qtbase5-dev-tools \
    wget \
    python3-pip \
  && pip3 install flask-socketio fastapi uvicorn \
  && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys D2486D2DD83DB69272AFE98867170598AF249743

# setup sources.list
RUN . /etc/os-release \
    && echo "deb http://packages.osrfoundation.org/gazebo/$ID-stable `lsb_release -sc` main" > /etc/apt/sources.list.d/gazebo-latest.list

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
    && apt-get install -y ignition-fortress clang clang-tools lld llvm-dev libc++-12-dev python3-colcon-mixin \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -yr \
    && rm -rf /var/lib/apt/lists/*

RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && colcon mixin update default || true
#-----------------
# Stage 2 - build
#-----------------

# compile rmf_demo_panel gui
# use wget
# RUN npm install --prefix src/demonstrations/rmf_demos/rmf_demos_panel/rmf_demos_panel/static/ \
  # && npm run build --prefix src/demonstrations/rmf_demos/rmf_demos_panel/rmf_demos_panel/static/

# colcon compilation
RUN export CXX=clang++  && export CC=clang && . /opt/ros/$ROS_DISTRO/setup.sh \
  && colcon build --mixin lld release 
    # --packages-skip-up-to rmf_demos_ign ros_ign_bridge ros_ign_image ros_ign_gazebo_demos ros_ign_gazebo ros_ign_interfaces

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
