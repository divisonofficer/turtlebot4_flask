#!/bin/bash

while true
do
  echo "HDR Trigger 전송!"
  ros2 action send_goal /jai_hdr_trigger jai_rosbridge/action/HDRTrigger "{space_id: '1234'}"
  sleep 4.6
done
