#!/bin/bash

# finish slam
rosservice call /finish_trajectory 0

# make pbstream
rosservice call /write_state "{filename: '/home/ev/cartographer/src/cartographer_ros/cartographer_ros/map/maps3.pbstream'}"

               
# pbstream to map  
rosrun cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename=/home/ev/cartographer/src/cartographer_ros/cartographer_ros/map/maps3.pbstream -map_filestem=/home/ev/cartographer/src/cartographer_ros/cartographer_ros/map

