#!/bin/bash

### BEGIN INIT INFO
# Provides:          watchDog
# Required-Start:    $remote_fs $syslog
# Required-Stop:     $remote_fs $syslog
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: start watchDog
# Description:       start watchDog
### END INIT INFO

sec=1
cnt=0
PROC_NAME=component_conta
Thread=`ps -ef | grep ${PROC_NAME} | grep -v "grep"`
source /opt/ros/humble/setup.bash
source /home/rm/Desktop/XMU-RCS-RM-VISION/install/setup.bash
ros2 launch rm_vision_bringup vision_bringup.launch.py
echo "Thread starting"

while [ 1 ]
do
count=`ps -ef | grep $PROC_NAME | grep -v "grep" | wc -l`
echo "Thread count: $count"
if [ $count -gt 1 ]; then
  echo "The $PROC_NAME is still alive!"
  sleep $sec
else
  echo "Starting $PROC_NAME"
  source /opt/ros/humble/setup.bash
  source /home/rm/Desktop/XMU-RCS-RM-VISION/install/setup.bash
  ros2 launch rm_vision_bringup vision_bringup.launch.py
  echo "$PROC_NAME has started!"
  sleep $sec
fi
done

exit 0
