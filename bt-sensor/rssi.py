#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int8
import sys

import bluetooth
import bluetooth._bluetooth as bt
import struct
import array
import fcntl


class BluetoothRSSI(object):
    """Object class for getting the RSSI value of a Bluetooth address.
    Reference: https://github.com/dagar/bluetooth-proximity
    """
    def __init__(self, addr):
        self.addr = addr
        self.hci_sock = bt.hci_open_dev()
        self.hci_fd = self.hci_sock.fileno()
        self.bt_sock = bluetooth.BluetoothSocket(bluetooth.L2CAP)
        self.bt_sock.settimeout(10)
        self.connected = False
        self.cmd_pkt = None

    def prep_cmd_pkt(self):
        """Prepares the command packet for requesting RSSI"""
        reqstr = struct.pack(
            "6sB17s", bt.str2ba(self.addr), bt.ACL_LINK, b"\0" * 17)
        request = array.array("B", reqstr)
        handle = fcntl.ioctl(self.hci_fd, bt.HCIGETCONNINFO, request, 1)
        handle = struct.unpack("8xH14x", request.tostring())[0]
        self.cmd_pkt = struct.pack('H', handle)

    def connect(self):
        """Connects to the Bluetooth address"""
        self.bt_sock.connect_ex((self.addr, 1))  # PSM 1 - Service Discovery
        self.connected = True

    def get_rssi(self):
        """Gets the current RSSI value.
        @return: The RSSI value (float) or None if the device connection fails
                 (i.e. the device is nowhere nearby).
        """
        try:
            # Only do connection if not already connected
            if not self.connected:
                self.connect()
            if self.cmd_pkt is None:
                self.prep_cmd_pkt()
            # Send command to request RSSI
            rssi = bt.hci_send_req(
                self.hci_sock, bt.OGF_STATUS_PARAM,
                bt.OCF_READ_RSSI, bt.EVT_CMD_COMPLETE, 4, self.cmd_pkt)
            resp = bytes([rssi[3]])
            rssi = struct.unpack('b', resp)[0]
            return rssi
        except IOError:
            # Happens if connection fails (e.g. device is not in range)
            self.connected = False
            return None

def talker(btad_list):
    rospy.init_node('bluetooth_rssi', anonymous=True)
    pub_list = []
    btobj_list = []
    for i,btad in enumerate(btad_list):
        pub_list.append(rospy.Publisher("duck{}".format(i), Int8, queue_size=10))
        btobj_list.append(BluetoothRSSI(addr=btad))
    rate = rospy.Rate(10)
    # rssi_list = []
    # FUTURE: send last rssi instead of 0 values
    while not rospy.is_shutdown():
        for i,pub in enumerate(pub_list):
            rssi = btobj_list[i].get_rssi()
            rospy.loginfo(rssi)
            if rssi is None:
                raise(ValueError('rssi is None'))
            pub.publish(Int8(rssi))
            rate.sleep()

if __name__ == '__main__':
    btad_list = [i for i in sys.argv[1:]]
    try:
        talker(btad_list)
    except rospy.ROSInterruptException:
        pass

