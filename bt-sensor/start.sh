#!/bin/bash
echo Other Macs: $MACS
./discover.sh
echo Our Mac:
hciconfig
export ROS_HOSTNAME=`hostname`
./rssi.py $MACS
