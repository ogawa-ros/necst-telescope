#!/usr/bin/env python3

name = "azel2refracted_raster"

import time
import rospy
import threading
import sys
import datetime
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool

class azel2refracted_raster(object):
    def __init__(self):
        rospy.Subscriber('/necst/telescope/coordinate/azel_raster_cmd',Float64MultiArray,self.recieve_azel)
        self.pub_real_azel = rospy.Publisher('/necst/telescope/coordinate/refracted_azel_cmd', Float64MultiArray, queue_size=1)
        self.pub_raster_check = rospy.Publisher('/necst/telescope/coordinate/raster_check', Bool, queue_size=1)


    def publish(self,q):
        x = q.data[0]
        y = q.data[1]
        lx = q.data[2]
        ly = q.data[3]
        scan_t = q.data[4]

        length = (lx**2+ly**2)**(1/2)
        dl = length/scan_t * 0.1
        dx = dl * lx/length
        dy = dl * ly/length
        num = int(length/dl)
        t0 = time.time()
        self.pub_raster_check.publish(True)

        for i in range(num):
            obstime = t0 + 0.1*i
            az = x + dx*i
            el = y + dy*i
            array = Float64MultiArray()
            array.data = [obstime, az, el]
            self.pub_real_azel.publish(array)
            time.sleep(0.01)

        while obstime < time.time():
            time.sleep(0.1)
            continue
        self.pub_raster_check.publish(False)
        return



if __name__ == "__main__":
    rospy.init_node(name)
    azel = azel2refracted_raster()
    rospy.spin()
