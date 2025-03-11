#!/bin/bash
# 启动多点导航
gnome-terminal \
--tab -e "zsh -c 'source $HOME/jetauto_ws/.zshrc;sudo systemctl stop start_app_node;killall -9 rosmaster;roslaunch jetauto_navigation navigation.launch map:=explore robot_name:=/ master_name:=/ & sleep 10;roslaunch jetauto_navigation publish_point.launch enable_navigation:=false robot_name:=/ master_name:=/ & rviz rviz -d  $HOME/jetauto_ws/src/jetauto_navigation/rviz/navigation_desktop.rviz'"
