#!/bin/bash
# Set Address
echo "Setting MAC with ID: $ID"
bccmd -d hci0  psset  bdaddr 01 00 $ID 01 01 00 01 01
bccmd -d hci0 warmreset
/etc/init.d/bluetooth restart

# Put bt in scanning mode
hciconfig hci0 piscan
