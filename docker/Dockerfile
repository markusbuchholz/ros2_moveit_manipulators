#Markus Buchholz

FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV IGNITION_VERSION=fortress

RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    build-essential \
    python3-pip \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-vcstool \
    git \
    wget \
    vim \
    nano \
    less && \
    apt-get install -y python3-rosdep

RUN rm -rf /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep init && \
    rosdep update

# Update system and install Cyclone DDS
RUN apt-get update && apt-get dist-upgrade -y && \
    apt-get install -y ros-humble-rmw-cyclonedds-cpp

RUN if ! colcon mixin list | grep -q 'default'; then \
    colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml; \
    fi && colcon mixin update default

RUN mkdir -p /root/ws_moveit2/src

RUN cd /root/ws_moveit2/src && \
    git clone --branch humble https://github.com/ros-planning/moveit2_tutorials && \
    vcs import < /root/ws_moveit2/src/moveit2_tutorials/moveit2_tutorials.repos

RUN cd /root/ws_moveit2 && \
    apt-get update && \
    rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

RUN apt-get update && \
    apt-get install -y ros-humble-image-transport-plugins libgflags-dev

RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && cd /root/ws_moveit2 && colcon build --mixin release"

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc && \
    echo "source /root/ws_moveit2/install/setup.bash" >> /root/.bashrc

RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc

WORKDIR /root/ws_moveit2

EXPOSE 11345
EXPOSE 22

RUN apt-get update && apt-get install -y lsb-release gnupg
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list
RUN apt-get update && apt-get install -y ignition-fortress

RUN apt-get update \
 && apt-get -y --quiet --no-install-recommends install \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
  && rm -rf /var/lib/apt/lists/*
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV QT_X11_NO_MITSHM=1

# Install additional ROS 2 packages
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros2-control \
    ros-humble-control* \
    ros-humble-gazebo-plugins \
    ros-humble-moveit-visual-tools \
    ros-humble-graph-msgs

RUN pip3 install numpy==1.23.5

RUN pip3 install pyroboplan

RUN pip3 install pybullet

RUN apt update && apt install -y ros-humble-pinocchio

CMD ["bash"]
