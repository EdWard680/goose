#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int8
import sys
import os

import bluetooth
import bluetooth._bluetooth as bt
import struct
import array
import fcntl

def read_inquiry_mode(sock):
    """returns the current mode, or -1 on failure"""
    # save current filter
    old_filter = sock.getsockopt( bt.SOL_HCI, bt.HCI_FILTER, 14)

    # Setup socket filter to receive only events related to the
    # read_inquiry_mode command
    flt = bt.hci_filter_new()
    opcode = bt.cmd_opcode_pack(bt.OGF_HOST_CTL, 
            bt.OCF_READ_INQUIRY_MODE)
    bt.hci_filter_set_ptype(flt, bt.HCI_EVENT_PKT)
    bt.hci_filter_set_event(flt, bt.EVT_CMD_COMPLETE);
    bt.hci_filter_set_opcode(flt, opcode)
    sock.setsockopt( bt.SOL_HCI, bt.HCI_FILTER, flt )

    # first read the current inquiry mode.
    bt.hci_send_cmd(sock, bt.OGF_HOST_CTL, 
            bt.OCF_READ_INQUIRY_MODE )

    pkt = sock.recv(255)

    status,mode = struct.unpack("xxxxxxBB", pkt)
    if status != 0: mode = -1

    # restore old filter
    sock.setsockopt( bt.SOL_HCI, bt.HCI_FILTER, old_filter )
    return mode

def write_inquiry_mode(sock, mode):
    """returns 0 on success, -1 on failure"""
    # save current filter
    old_filter = sock.getsockopt( bt.SOL_HCI, bt.HCI_FILTER, 14)

    # Setup socket filter to receive only events related to the
    # write_inquiry_mode command
    flt = bt.hci_filter_new()
    opcode = bt.cmd_opcode_pack(bt.OGF_HOST_CTL, 
            bt.OCF_WRITE_INQUIRY_MODE)
    bt.hci_filter_set_ptype(flt, bt.HCI_EVENT_PKT)
    bt.hci_filter_set_event(flt, bt.EVT_CMD_COMPLETE);
    bt.hci_filter_set_opcode(flt, opcode)
    sock.setsockopt( bt.SOL_HCI, bt.HCI_FILTER, flt )

    # send the command!
    bt.hci_send_cmd(sock, bt.OGF_HOST_CTL, 
            bt.OCF_WRITE_INQUIRY_MODE, struct.pack("B", mode) )

    pkt = sock.recv(255)

    status = struct.unpack("xxxxxxB", pkt)[0]

    # restore old filter
    sock.setsockopt( bt.SOL_HCI, bt.HCI_FILTER, old_filter )
    if status != 0: return -1
    return 0

def get_rssis(addrs):
    try:
        sock = bt.hci_open_dev()
    except:
        print("error accessing bluetooth device...")
        sys.exit(1)

    try:
        mode = read_inquiry_mode(sock)
    except Exception as e:
        print("error reading inquiry mode.  ")
        print("Are you sure this a bluetooth 1.2 device?")
        print(e)
        sys.exit(1)

    if mode != 1:
        try:
            result = write_inquiry_mode(sock, 1)
        except Exception as e:
            print("error writing inquiry mode.  Are you sure you're root?")
            print(e)
            sys.exit(1)
        if result != 0:
            print("error while setting inquiry mode")

    # save current filter
    old_filter = sock.getsockopt( bt.SOL_HCI, bt.HCI_FILTER, 14)

    # perform a device inquiry on bluetooth device #0
    # The inquiry should last 8 * 1.28 = 10.24 seconds
    # before the inquiry is performed, bt should flush its cache of
    # previously discovered devices
    flt = bt.hci_filter_new()
    bt.hci_filter_all_events(flt)
    bt.hci_filter_set_ptype(flt, bt.HCI_EVENT_PKT)
    sock.setsockopt( bt.SOL_HCI, bt.HCI_FILTER, flt )
    
    
    while True:
        duration = 4
        max_responses = 255
        cmd_pkt = struct.pack("BBBBB", 0x33, 0x8b, 0x9e, duration, max_responses)
        bt.hci_send_cmd(sock, bt.OGF_LINK_CTL, bt.OCF_INQUIRY, cmd_pkt)

        results = []

        done = False
        while not done:
            pkt = sock.recv(255)
            ptype, event, plen = struct.unpack("BBB", pkt[:3])
            if event == bt.EVT_INQUIRY_RESULT_WITH_RSSI:
                pkt = pkt[3:]
                nrsp = bluetooth.get_byte(pkt[0])
                for i in range(nrsp):
                    addr = bt.ba2str( pkt[1+6*i:1+6*i+6] )
                    if addr in addrs:
                        rssi = bluetooth.byte_to_signed_int(
                                bluetooth.get_byte(pkt[1+13*nrsp+i]))
                        yield ( addr, rssi )
            elif event == bt.EVT_INQUIRY_COMPLETE:
                done = True
            elif event == bt.EVT_CMD_STATUS:
                status, ncmd, opcode = struct.unpack("BBH", pkt[3:7])
                if status != 0:
                    print("uh oh...")
                    printpacket(pkt[3:7])
                    done = True
                    sock.setsockopt( bt.SOL_HCI, bt.HCI_FILTER, old_filter )
                    sys.exit(1)
    
    
    # restore old filter
    sock.setsockopt( bt.SOL_HCI, bt.HCI_FILTER, old_filter )

def talker(btad_list, this_duck):
    rospy.init_node('bluetooth_rssi', anonymous=True)
    pub_list = []
    for i,btad in enumerate(btad_list):
        pub_list.append(rospy.Publisher("duck{}/rssi/duck{}".format(this_duck, btad[-1]), Int8, queue_size=10))
    
    print("Starting inquiry...")
    for (addr, rssi) in get_rssis(btad_list):
        if rospy.is_shutdown():
            return 
        
        btid = btad_list.index(addr)
        rospy.loginfo("{}: {}".format(addr, rssi))
        pub_list[btid].publish(Int8(rssi))

if __name__ == '__main__':
    btad_list = [i for i in sys.argv[1:]]
    this_duck = os.environ.get("ID") or -1
    try:
        talker(btad_list, this_duck)
    except rospy.ROSInterruptException:
        pass

