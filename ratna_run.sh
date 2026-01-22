#!/bin/bash

# Launch Gazebo
gnome-terminal --tab --title="Gazebo" -- bash -c "source install/setup.bash;
                                 ros2 launch ratna_description gazebo.launch.py gazebo_rviz:=false;
                                echo Press any key to close;
                                read -n 1"

gnome-terminal --tab --title="Behavior Tree" -- bash -c "source install/setup.bash;
                                 ros2 run ratna_control_pkg ratna_control;
                                echo Press any key to close;
                                read -n 1"