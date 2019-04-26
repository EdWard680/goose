#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int8
from tkinter import *
import threading
import time
import atexit

FLAG = 0

RSSI = -30
RSSI_SHIFT = 5
RSSI_SPIKE = 20
PERIOD = 0.5

class Window(Frame):
    def __init__(self, parent=None, ducks=2):
        Frame.__init__(self, parent)
        self.parent = parent
        self.ducks = ducks
        self.init_window()

    def init_window(self):
        self.parent.title("Simulated RSSI Values")
        self.pack(fill=BOTH, expand=1)

    def increase_rssi(self,L):
        global RSSI
        RSSI += RSSI_SHIFT
        L.configure(text="RSSI: {}".format(RSSI))

    def decrease_rssi(self,L):
        global RSSI
        RSSI -= RSSI_SHIFT
        L.configure(text="RSSI: {}".format(RSSI))
        
    def positive_spike_rssi(self, L):
        global RSSI
        RSSI += RSSI_SPIKE
        L.configure(text="RSSI: {}".format(RSSI))
        self.parent.update()
        self.parent.after(int(PERIOD*1000))
        RSSI -= RSSI_SPIKE
        L.configure(text="RSSI: {}".format(RSSI))

    def negative_spike_rssi(self, L):
        global RSSI
        RSSI -= RSSI_SPIKE
        L.configure(text="RSSI: {}".format(RSSI))
        self.parent.update()
        self.parent.after(int(PERIOD*1000))
        RSSI += RSSI_SPIKE
        L.configure(text="RSSI: {}".format(RSSI))

    def quit(self):
        global FLAG
        FLAG = 1
        exit()



def simulate(ducks):
    a,b = sorted(ducks)
    pub = rospy.Publisher('rssi/{}_{}'.format(a,b), Int8, queue_size=10)
    while not rospy.core.is_shutdown():
        pub.publish(Int8(RSSI))
        rospy.rostime.wallsleep(PERIOD)
        if FLAG:
            break

def at_exit(t):
    t.join()

if __name__ == '__main__':
    ducks = sys.argv[1:] 

    rospy.init_node("simulation")

    root = Tk()
    w = Window(root, ducks)
    w.pack()

    L = Label(w, text="RSSI: {}".format(RSSI))
    L.pack()
    L.place()

    increase_rssi = Button(w, text="Increase RSSI", command=lambda:w.increase_rssi(L))
    increase_rssi.pack()
    increase_rssi.place()

    decrease_rssi = Button(w, text="Decrease RSSI", command=lambda:w.decrease_rssi(L))
    decrease_rssi.pack()
    decrease_rssi.place()

    positive_spike_rssi = Button(w, text="Positive RSSI Spike", command=lambda:w.positive_spike_rssi(L))
    positive_spike_rssi.pack()
    positive_spike_rssi.place()

    negative_spike_rssi = Button(w, text="Negative RSSI Spike", command=lambda:w.negative_spike_rssi(L))
    negative_spike_rssi.pack()
    negative_spike_rssi.place()

    quit_button = Button(w, text='Quit', command=w.quit, anchor=NW)
    quit_button.pack()
    quit_button.place()

    t = threading.Thread(target=simulate, args=[ducks])
    t.start()


    atexit.register(at_exit, t)

    root.mainloop()


