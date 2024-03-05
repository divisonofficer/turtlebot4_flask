#!/bin/bash

## kill node 


#step 1 : get the node name as arguemnt

node=$1

#step 2 : check if the node name is empty
if [ -z "$node" ]
then
    echo "node name is empty"
    exit 1
fi

#step 3 : check if the node is already running
ps aux | grep ros2 | grep $node
if [ $? -ne 0 ]
then
    echo "node $node is not running"
    exit 1
fi

#step 4 : kill the node
kill -9 $(ps aux | grep ros2 | grep $node | awk '{print $2}')
echo "node $node killed"