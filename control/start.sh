#!/bin/bash
source ../devel/setup.bash
VEHICLE_NAME=`hostname`
roslaunch goose_control/src/control.launch ducks:="${IDS}" goose:="${GOOSE}" veh:="${VEHICLE_NAME}"

echo "what the HECK"
