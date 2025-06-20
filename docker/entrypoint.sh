#!/bin/bash
set -e

# setup ros2 environment
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]
then
  source "/opt/ros/$ROS_DISTRO/setup.bash" --
elif [ -f /opt/ros/install/setup.bash ]
then
  source /opt/ros/install/setup.bash --
fi

if [ -f /root/ros2_ws/install/setup.bash ]
then
  source /root/ros2_ws/install/setup.bash
fi

exec "$@"
