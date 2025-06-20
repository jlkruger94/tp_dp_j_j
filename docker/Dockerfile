# syntax=docker/dockerfile:1
ARG ROS_DISTRO=jazzy
 
FROM osrf/ros:${ROS_DISTRO}-desktop AS base
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

RUN set -euo pipefail
ENV DEBIAN_FRONTEND=noninteractive

# Get Ubuntu packages
RUN apt-get update && apt-get install -y \
    build-essential \
    curl \
    tmux \
    git \
    vim \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN apt-get update -y && apt-get install -y nano

# Create workspace
RUN mkdir -p /root/ros2_ws
WORKDIR /root/ros2_ws/src
# Copy the entire src directory from the root of the workspace
COPY ./src /root/ros2_ws/src

# Install dependencies
RUN apt-get update -y && apt-get install -y \
  #ros-${ROS_DISTRO}-moveit \
  ros-${ROS_DISTRO}-gz-tools-vendor \
  ros-${ROS_DISTRO}-gz-sim-vendor \
  ros-${ROS_DISTRO}-ros-gz-sim \
  ros-${ROS_DISTRO}-ros-gz-bridge \ 
  ros-${ROS_DISTRO}-xacro \
  ros-${ROS_DISTRO}-gz-ros2-control \
  ros-${ROS_DISTRO}-joint-state-publisher-gui \
  ros-${ROS_DISTRO}-joint-state-broadcaster \
  ros-${ROS_DISTRO}-ros2-control \
  ros-${ROS_DISTRO}-ros2-controllers \
  ros-${ROS_DISTRO}-plotjuggler-ros

# See https://gazebosim.org/docs/latest/ros2_gz_vendor_pkgs/
  

# Compile the workspace
WORKDIR /root/ros2_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
  && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
  && colcon build --symlink-install

# Configure entrypoint
COPY ./docker/entrypoint.sh /
RUN chmod +x  /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]
CMD ["bash"]
