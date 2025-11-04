#!/bin/bash

# Exit if any command fails
set -e

# Source ROS and ROS workspaces
source /opt/ros/melodic/setup.bash
source ~/darknet_ws/devel/setup.bash
source ~/agv_tester/devel/setup.bash

# Set ROS master URI and ROS hostname
export ROS_MASTER_URI=http://nano-4gb-jp451:11311
export ROS_HOSTNAME=nano-4gb-jp451

# Export CUDA libraries
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH

# Function to wait for a specific ROS topic
wait_for_ros_topic() {
    local topic=$1
    echo "Waiting for topic: $topic"
    until rostopic list 2>/dev/null | grep -q "$topic"; do
        sleep 1
    done
    echo "Topic $topic is available"
}

# --- First launch - object detection launch file ---
echo "Launching object detection launch file"
roslaunch agv_tester_bringup detected_objects_sender.launch &
PID1=$!

# Wait for /darknet_ros/detection_image ROS topic to appear
wait_for_ros_topic "/darknet_ros/detection_image"
echo "Object detection launch is running in background (PID=$PID1)"

# ---Restart nvargus-daemon to be able to launch camera node---
echo "Restarting nvargus-daemon"
sudo systemctl restart nvargus-daemon
sleep 3

# --- Second launch - camera image launch file ---
echo "Launching camera image launch file"
roslaunch agv_tester_bringup camera_image_sender.launch &
PID2=$!

# Wait for /csi_cam_0/image_raw/compressed ROS topic to appear
wait_for_ros_topic "/csi_cam_0/image_raw/compressed"
echo "Camera image launch is running in background (PID=$PID2)"

# --- Third launch - vehicle drive control launch file ---
echo "Launching vehicle drive control launch file"
roslaunch agv_tester_bringup vehicle_drive_control.launch &
PID3=$!

echo "Vehicle drive control launch is running in background (PID=$PID3)"

# ---Keep script alive so backgrounded roslaunches don’t die---
wait $PID1 $PID2 $PID3

