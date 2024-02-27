#!/bin/bash

remote_ip="10.42.0.1"
remote_folder="ubuntu@$remote_ip:~/turtlebot4_ws/control"


# Step 2: Copy files to remote folder
# if ros_call.py is not in folder, exit
if [ ! -f "./ros_call.py" ]; then
    echo "ros_call.py not found in folder"
    exit 1
fi
scp -r ./* $remote_folder

