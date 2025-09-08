#!/bin/bash


sudo sh -c "echo '$(pwd)/install/yesense_std_ros2/lib/yesense_std_ros2/serial' >> /etc/ld.so.conf"

sudo ldconfig
