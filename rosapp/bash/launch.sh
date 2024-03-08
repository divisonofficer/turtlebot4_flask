#!/bin/bash



#step 1 : get the package name & node name as arguemnt
package_name=$1
node=$2
LOG="/tmp/ros2_launch/log/"

#step 2 : check if the package name is empty
if [ -z "$package_name" ]
then
    echo "package name is empty"
    exit 1
fi

#step 3 : check if the node name is empty
if [ -z "$node" ]
then
    echo "node name is empty"
    exit 1
fi

#step 4 : check if the package name exists in ros2 pkg list

ros2 pkg list | grep $package_name
if [ $? -ne 0 ]
then
    echo "package $package_name not found"
    exit 1
fi

#step 5 : check if the node name exists in the package executable list
ls /opt/ros/humble/share/$package_name/launch | grep $node
if [ $? -ne 0 ]
then
    echo "node $node not found in package $package_name"
    exit 1
fi

#step 6 : check if the node is already running
ps aux | grep ros2 | grep $node
if [ $? -eq 0 ]
then
    echo "node $node is already running"
    exit 1
fi


#step 7 : launch the node
# if log folder does not exist, create it
if [ ! -d $LOG ]; then
    mkdir -p $LOG
fi

touch $LOG$node.log

#step 8 : if launch option is provided, use it
if [ -z "$3" ]
then
    nohup ros2 launch $package_name $node.launch.py > $LOG$node.log 2>&1 &
else
    nohup ros2 launch $package_name $node.launch.py $3 > $LOG$node.log 2>&1 &
fi