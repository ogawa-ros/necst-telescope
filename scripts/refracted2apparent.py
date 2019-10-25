#!/usr/bin/env python3

name = "refracted2apparent"
kisa_path = "./kisa.data"

import math
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray


class refracted2apparent(object):
    azel = ""
    a1 = 0
    a2 = 0
    a3 = 0
    b1 = 0
    b2 = 0
    b3 = 0
    g1 = 0

    def __init__(self):
        self.read_kisa()
        pub_az = rospy.Publisher("/necst-telescope/coordinate/apparent_az_cmd",Foat64, queue_size=1)
        pub_el = rospy.Publisher("/necst-telescope/coordinate/apparent_el_cmd", Float64, queue_size=1)
        rospy.Subscriber('/necst-telescope/coordinate/refracted_azel_cmd', Float64MultiArray, self.recieve_azel)

    def recieve_azel(self, array):
        self.azel = array.data

    def read_kisa(self):
        fkisa = open(kisa_path,"r")
        kisa = fkisa.readlines()
        self.a1 = kisa[0]
        self.a2 = kisa[1]
        self.a3 = kisa[2]
        self.b1 = kisa[3]
        self.b2 = kisa[4]
        self.b3 = kisa[5]
        self.g1 = kisa[6]

    def calculate_offset(self):
        while not rospy.is_shutdown():
            if self.azel != '':
                az = math.radians(self.azel[1])
                el = math.radians(self.azel[2])

                cos_az = math.cos(az)
                sin_az = math.sin(az)
                cos_el = math.cos(el)
                sin_el = math.sin(el)
                pi = math.pi

                d_az =  self.a1*sin_el + self.a2 + self.a3*cos_el + self.b1*sin_az*sin_el - self.b2*cos_az*sin_el
                d_el =  self.b1*cos_az + self.b2*sin_az + self.b3 + self.g1*el*180./pi
                ### convert to encoder offset on the horizon ###
                d_az =  d_az / cos_el

                ### apply the correction values ->  radians  ###
                az2 = az +(d_az /60.0)*pi / 180.
                el2 = el +(d_el /60.0)*pi / 180.

                self.pub_az.publish(az2)
                self.pud_el.publish(el2)
            else:
                pass
            continue

    def start_thread(self):
        th = threading.Thread(target=self.calculate_offset)
        th.setDaemon(True)
        th.start()


if __name__ == "__main__":
    rospy.init_node(name)
    azel = refracted2apparent()
    azel.start_thread()
