#!/bin/bash
source ../devel/setup.bash
roslaunch goose_control/src/control.launch ducks:="${IDS}" goose:="${GOOSE}"
