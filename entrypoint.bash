#!/bin/bash

source /opt/ros/jazzy/setup.bash
source ws/install/setup.bash

ros2 run rmw_zenoh_cpp rmw_zenohd &
sleep 2
ros2 launch jackal_launch jackal.composition.launch.py
