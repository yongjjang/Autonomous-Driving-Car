#!/usr/bin/env bash

echo "setting GAZEBO_RESOURCE_PATH to $(pwd)/world"
export GAZEBO_RESOURCE_PATH=$(pwd)/world:$GAZEBO_RESOURCE_PATH
echo "setting GAZEBO_PLUGIN_PATH to $(pwd)/build"
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$(pwd)/build
