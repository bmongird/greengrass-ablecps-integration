#!/usr/bin/env bash

cp -r $ALC_HOME/bluerov2_standalone/catkin_ws ./bluerov_src

docker-compose build

rm -r ./bluerov_src
