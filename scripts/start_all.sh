#!/bin/bash

echo "[1/4] Stopping existing processes and freeing ports 8765 (Foxglove), 1101/1102 (UnetStack)..."
killall -9 foxglove_bridge joy_node python3 java 2>/dev/null
sudo fuser -k 8765/tcp 2>/dev/null
sudo fuser -k 1101/tcp 2>/dev/null
sudo fuser -k 1102/tcp 2>/dev/null

echo "[2/4] Cleaning previous workspace and UnetStack logs..."
rm -rf ~/rov_ws/log/*
rm -rf ~/unetstack/unet-community-3.4.4/unet-3.4.4/logs/* 2>/dev/null

echo "[3/4] Starting UnetStack acoustic simulator in background..."
(cd ~/unetstack/unet-community-3.4.4/unet-3.4.4 && bin/unet samples/2-node-network.groovy &)
sleep 4

echo "[4/4] Launching ROS 2 ROV simulation stack (rov.launch.py)..."
cd ~/rov_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rov_brain rov.launch.py
