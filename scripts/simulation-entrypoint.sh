#!/bin/bash
source /opt/ros/humble/setup.bash
source /usr/share/gazebo-11/setup.sh
source /home/robomaker/workspace/simulation_ws/install/setup.sh
source ./install/setup.sh

#export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(rospack find mini_pupper_simulation)/worlds/meshes
printenv

exec "${@:1}"
