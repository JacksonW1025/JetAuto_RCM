#!/bin/bash
# 启动gmapping建图
gnome-terminal \
--tab -e "zsh -c 'source $HOME/jetauto_ws/.zshrc;sudo systemctl stop start_app_node;killall -9 rosmaster;roslaunch jetauto_slam slam.launch robot_name:=/ master_name:=/'" \
--tab -e "zsh -c 'source $HOME/jetauto_ws/.zshrc;sleep 30;roscd jetauto_slam/rviz;rviz rviz -d gmapping_desktop.rviz'" \
--tab -e "zsh -c 'source $HOME/jetauto_ws/.zshrc;sleep 30;roslaunch jetauto_peripherals teleop_key_control.launch robot_name:=/'" \
--tab -e "zsh -c 'source $HOME/jetauto_ws/.zshrc;sleep 30;rosrun jetauto_slam map_save.py'"
