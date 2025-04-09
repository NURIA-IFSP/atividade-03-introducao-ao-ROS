#!/bin/bash

export DISPLAY=:1
export HOME=/home/${USER:-ubuntu}

# Inicia terminal principal
xfce4-terminal --geometry=100x30+100+100 --title="Main Terminal" &

# Terminal com ROS (exemplo)
xfce4-terminal --geometry=80x24+600+100 --title="ROS Tools" -e "bash -c 'source /opt/ros/noetic/setup.bash'" &

# Inicia VSCode apontando para o projeto
code /catkin_ws/src/ &
