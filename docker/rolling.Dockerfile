#-----------------------
# Stage 1 - Dependencies
#-----------------------

FROM ros:humble AS builder

RUN apt-get update \
  && apt-get install -y \
    cmake \
    curl \
    git \
    python3-colcon-common-extensions \
    python3-vcstool \
    wget \
    python3-pip \
    clang lldb lld \
  && pip3 install flask-socketio fastapi uvicorn \
  && update-alternatives --install /usr/bin/c++ c++ /usr/bin/clang++ 100 \
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

RUN wget https://raw.githubusercontent.com/open-rmf/rmf/main/rmf.repos
RUN vcs import src < rmf.repos \
    && apt-get update \
    && apt-get upgrade -y \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -yr \
    && rm -rf /var/lib/apt/lists/*

#-----------------
# Stage 2 - build
#-----------------

# compile rmf_demo_panel gui
# use wget
# RUN npm install --prefix src/demonstrations/rmf_demos/rmf_demos_panel/rmf_demos_panel/static/ \
  # && npm run build --prefix src/demonstrations/rmf_demos/rmf_demos_panel/rmf_demos_panel/static/

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
