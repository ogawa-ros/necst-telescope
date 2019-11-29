#!/usr/bin/env python3

name = "azel2refracted"

import time
import rospy
import threading
import sys
import datetime
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool


class azel2refracted(object):
    def __init__(self):
        rospy.Subscriber('/necst/telescope/coordinate/stop_refracted_cmd' ,Bool, self.recieve_stop_cmd)
        rospy.Subscriber('/necst/telescope/coordinate/azel_cmd',Float64MultiArray,self.recieve_azel)

        self.pub_real_azel = rospy.Publisher('/necst/telescope/coordinate/refracted_azel_cmd', Float64MultiArray, queue_size=1)

    def recieve_azel(self,q):
        self.azel = q.data

    def recieve_stop_cmd(self,q):
        if q.data == True:
            self.azel = ''
        else:
            pass

    def publish_azel(self):
        while not rospy.is_shutdown():
            if self.azel_cmd != '':
                alt = self.el_cmd
                az = self.az_cmd
                array = Float64MultiArray()
                array.data = [az, alt]
                self.pub_real_azel.publish(array)
                time.sleep(0.1)
            else:
                time.sleep(0.1)
            continue

    def start_thread(self):
        th = threading.Thread(target=self.publish_azel)
        th.setDaemon(True)
        th.start()

if __name__ == "__main__":
    rospy.init_node(name)
    azel = azel2refracted()
    azel.start_thread()
