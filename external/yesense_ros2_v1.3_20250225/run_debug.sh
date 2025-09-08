#!/bin/bash

#


# setup env
. install/setup.bash

# 直接运行可执行文件
ros2 run --prefix 'gdb -ex run --args' yesense_std_ros2 yesense_node_publisher

# 使用 launch 文件运行
#roslaunch yesense_imu run_without_rviz.launch
