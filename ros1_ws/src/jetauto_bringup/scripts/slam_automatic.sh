#!/bin/bash
# 启动explore自主建图
gnome-terminal \
--tab -e "zsh -c 'source $HOME/jetauto_ws/.zshrc;sudo systemctl stop start_app_node;killall -9 rosmaster;roslaunch jetauto_slam slam.launch slam_methods:=explore robot_name:=/ master_name:=/ & sleep 10;rviz rviz -d  $HOME/jetauto_ws/src/jetauto_slam/rviz/explore_desktop.rviz'"
