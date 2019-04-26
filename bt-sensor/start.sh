#!/bin/bash
echo Other Macs: $MACS
./discover.sh
echo Our Mac:
hciconfig
export ROS_HOSTNAME=`hostname`

while [ -z "$MACS" ]
do
	sleep  10
done

echo "Starting the scanner..."
./rssi.py $MACS
