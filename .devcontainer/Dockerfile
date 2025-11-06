# Use the official ROS 2 Jazzy desktop image as base
ARG BASE_IMAGE=osrf/ros:jazzy-desktop
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# Install development tools, RViz, and Gazebo (ros_gz integration)
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    build-essential \
    git \
    sudo \
    curl \
    wget \
    nano \
    vim \
    mesa-utils \
    mesa-vulkan-drivers \
    libvulkan1 \
    vulkan-tools \
    python3-vcstool \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-ros-gz-sim \
    python3-usb \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep (skip if already done)
RUN rosdep init || true && rosdep update || true

# Set workspace
WORKDIR /workspace

# Source ROS 2 environment automatically for every shell
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc \
 && echo "if [ -f /workspace/ros2_ws/install/setup.bash ]; then source /workspace/ros2_ws/install/setup.bash; fi" >> /etc/bash.bashrc

# Default command
CMD ["bash"]
