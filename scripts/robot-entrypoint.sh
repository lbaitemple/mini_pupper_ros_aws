#!/bin/bash

export PYTHONPATH=$PYTHONPATH:$DANCE_CONFIG:/usr/local/lib/python3.10/dist-packages

source /opt/ros_demos/setup.bash
source /opt/greengrass_bridge/setup.bash
source /opt/ros/humble/setup.bash


printenv


exec "${@:1}"
