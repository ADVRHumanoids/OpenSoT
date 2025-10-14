#!/bin/bash
set -e

# make forest https
export HHCM_FOREST_CLONE_DEFAULT_PROTO=https

# activate python venv
source env/bin/activate

# create ws and source it
sudo chown user:user test_ws test_ws/src
mkdir -p test_ws && cd test_ws
forest init

# setup env
source /opt/ros/jazzy/setup.bash
source setup.bash

# here we want to check that opensot builds and works against the binaries of the
# xbot dependencies
source /opt/xbot/setup.sh
sudo apt remove -y open_sot

# add recipes
forest add-recipes git@github.com:advrhumanoids/multidof_recipes.git -t ros2

# build
export PYTHONUNBUFFERED=1
forest grow OpenSoT --verbose --clone-depth 1 -j ${FOREST_JOBS:-1}

# build tests
cd build/OpenSoT
cmake -DOPENSOT_COMPILE_TESTS=1 .
make -j ${FOREST_JOBS:-1}