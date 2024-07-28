# Use the official ROS 2 base image
FROM osrf/ros:humble-desktop

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV IGNITION_VERSION=fortress

# Install necessary packages and dependencies
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

# Initialize rosdep and handle existing sources list
RUN rm -rf /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep init && \
    rosdep update

# Update system and install Cyclone DDS
RUN apt-get update && apt-get dist-upgrade -y && \
    apt-get install -y ros-humble-rmw-cyclonedds-cpp

# Set up colcon mixins, check if it exists before adding
RUN if ! colcon mixin list | grep -q 'default'; then \
    colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml; \
    fi && colcon mixin update default

# Create a colcon workspace
RUN mkdir -p /root/ws_moveit2/src

# Clone the MoveIt tutorials and import the rest of MoveIt
RUN cd /root/ws_moveit2/src && \
    git clone --branch humble https://github.com/ros-planning/moveit2_tutorials && \
    vcs import < /root/ws_moveit2/src/moveit2_tutorials/moveit2_tutorials.repos

# Install dependencies
RUN cd /root/ws_moveit2 && \
    apt-get update && \
    rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

# Manually install missing dependencies
RUN apt-get update && \
    apt-get install -y ros-humble-image-transport-plugins libgflags-dev

# Source ROS 2 setup script before building
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && cd /root/ws_moveit2 && colcon build --mixin release"

# Source the ROS 2 and workspace setup scripts
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc && \
    echo "source /root/ws_moveit2/install/setup.bash" >> /root/.bashrc

# Set Cyclone DDS as the default RMW implementation
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc

# Set the working directory
WORKDIR /root/ws_moveit2

# Expose ROS 2 ports
EXPOSE 11345
EXPOSE 22

# Install Gazebo Fortress
RUN apt-get update && apt-get install -y lsb-release gnupg
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list
RUN apt-get update && apt-get install -y ignition-fortress

# Install NVIDIA software
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

# Default command to run
CMD ["bash"]

