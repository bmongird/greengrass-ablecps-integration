#!/usr/bin/env bash
echo "Sourcing bashrc"
source ~/.bashrc
echo "ALC_HOME: $ALC_HOME"

cp -r $ALC_HOME/bluerov2_standalone/catkin_ws ./bluerov_src

docker-compose build

rm -r ./bluerov_src
