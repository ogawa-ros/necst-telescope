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
    azel_cmd = ""
    def __init__(self):
        rospy.Subscriber('/necst/telescope/coordinate/stop_cmd' ,Bool, self.recieve_stop_cmd)
        rospy.Subscriber('/necst/telescope/coordinate/azel_cmd',Float64MultiArray,self.recieve_azel)

        self.pub_real_azel = rospy.Publisher('/necst/telescope/coordinate/refracted_azel_cmd', Float64MultiArray, queue_size=1)

        self.init_flag  = True

    def recieve_azel(self,q):
        self.azel_cmd = q.data

    def recieve_stop_cmd(self,q):
        if q.data == True:
            self.azel_cmd = ''
            self.init_flag  = True
        else:
            pass

    def publish_azel(self):
        while not rospy.is_shutdown():
            if self.azel_cmd != '':
                if self.init_flag == True:
                    for i in range(5):
                        obstime = time.time()+ 0.2*i
                        az = self.azel_cmd[0] + self.azel_cmd[2]
                        alt  = self.azel_cmd[1] + self.azel_cmd[3]
                        array = Float64MultiArray()
                        array.data = [obstime, az, alt]
                        self.pub_real_azel.publish(array)
                    self.init_flag  = False

                else:
                    obstime = time.time()+ 1
                    az = self.azel_cmd[0] + self.azel_cmd[2]
                    alt= self.azel_cmd[1] + self.azel_cmd[3]
                    array = Float64MultiArray()
                    array.data = [obstime, az, alt]
                    self.pub_real_azel.publish(array)
                    time.sleep(0.2)

            else:
                time.sleep(0.0001)
            continue

    def start_thread(self):
        th = threading.Thread(target=self.publish_azel)
        th.setDaemon(True)
        th.start()

if __name__ == "__main__":
    rospy.init_node(name)
    azel = azel2refracted()
    azel.start_thread()
    rospy.spin()
