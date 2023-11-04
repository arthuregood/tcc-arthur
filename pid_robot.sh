#!/bin/bash

roscore &
rosrun stage_ros stageros ~/tcc-arthur/src/tcc/worlds/map_sonar.world &
rosrun tcc human.py robot_1 -5 5 &
rosrun tcc human.py robot_2 3 5 &

#rosrun tcc robot.py