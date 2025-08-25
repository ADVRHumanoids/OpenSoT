#!/bin/bash
set -e

# setup env
source /opt/ros/jazzy/setup.bash
source ~/test_ws/setup.bash 

# run tests
cd ~/test_ws/build/OpenSoT
ctest --output-on-failure