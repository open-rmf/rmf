FROM ros:humble AS builder

# dependencies
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

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
