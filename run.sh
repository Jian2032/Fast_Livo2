#!/bin/bash

# 进入你的工作空间路径
WORKSPACE_PATH=~/home/sun/ws_livox  # 替换为你的实际工作空间路径

# 启动 mvs_camera_trigger
gnome-terminal -- bash -c "
cd $WORKSPACE_PATH;
source devel/setup.bash;
roslaunch mvs_ros_driver mvs_camera_trigger.launch;
exec bash"

sleep 3  # 给前一个节点一些时间初始化

# 启动 Livox MID-360
gnome-terminal -- bash -c "
cd $WORKSPACE_PATH;
source devel/setup.bash;
roslaunch livox_ros_driver2 msg_MID360.launch;
exec bash"

sleep 3

# 启动 FAST-LIO2 mapping
gnome-terminal -- bash -c "
cd $WORKSPACE_PATH;
source devel/setup.bash;
roslaunch fast_livo mapping_mid360.launch;
exec bash"
