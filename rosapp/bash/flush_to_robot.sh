#!/bin/bash

remote_ip="192.168.185.3"
remote_folder="ubuntu@$remote_ip:~/turtlebot4_ws/turtlebot4_flask"
remote_password="turtlebot4"


# Step 2: Copy files to remote folder
# if ros_call.py is not in folder, exit
if [ ! -f "./controller_flask.py" ]; then
    echo "controller_flask.py not found in folder"
    exit 1
fi

# 

# Dont copy venv folder
# Dont copy reactapp folder

sshpass -p $remote_password rsync -av --exclude 'venv' --exclude 'reactapp' . $remote_folder

