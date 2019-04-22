#!/bin/bash
echo Other Macs: $MACS
./discover.sh
echo Our Mac:
hciconfig
export ROS_HOSTNAME=`hostname`
while true
do
    sleep 10
done
# ./rssi.py $MACS
