#!/bin/bash
# Set Address
echo "Setting MAC with ID: $ID"
bccmd -d $DEV_ID  psset  bdaddr 01 00 $ID 01 01 00 01 01
bccmd -d $DEV_ID warmreset
/etc/init.d/bluetooth restart

# Put bt in scanning mode
hciconfig $DEV_ID piscan
