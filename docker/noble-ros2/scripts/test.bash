#!/bin/bash
set -e

# setup env
source /opt/ros/jazzy/setup.bash
source ~/test_ws/setup.bash 
source /opt/xbot/setup.sh

# run tests
cd ~/test_ws/build/OpenSoT
ctest --output-on-failure
