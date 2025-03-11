#!/bin/bash

source $HOME/jetauto_ws/.typerc

export CUDA_HOME=/usr/local/cuda
export PATH=$CUDA_HOME/bin:$PATH
export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH

export HOST_IP=localhost
export MASTER_IP=localhost
export ROS_MASTER_URI=http://$MASTER_IP:11311
export ROS_HOSTNAME=$MASTER_IP

if [ $ZSH_VERSION ]; then
  . /opt/ros/melodic/setup.zsh
  . $HOME/jetauto_ws/devel/setup.zsh
elif [ $BASH_VERSION ]; then
  . /opt/ros/melodic/setup.bash
  . $HOME/jetauto_ws/devel/setup.bash
else
  . /opt/ros/melodic/setup.sh
  . $HOME/jetauto_ws/devel/setup.sh
fi
exec "$@"
