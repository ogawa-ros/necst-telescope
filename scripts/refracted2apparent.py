#!/usr/bin/env python3

name = "refracted2apparent"
kisa_path = "/home/exito/ros/src/necst-1p85m2019/lib/kisa.dat"

import math
import rospy
import threading
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
import time


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
        self.pub_az = rospy.Publisher("/necst/telescope/coordinate/apparent_az_cmd",Float64, queue_size=1)
        self.pub_el = rospy.Publisher("/necst/telescope/coordinate/apparent_el_cmd", Float64, queue_size=1)
        rospy.Subscriber('/necst/telescope/coordinate/refracted_azel_cmd', Float64MultiArray, self.calculate_offset)

    def recieve_azel(self, array):
        self.azel = array.data

    def read_kisa(self):
        fkisa = open(kisa_path,"r")
        kisa = fkisa.readlines()
        self.a1 = float(kisa[0])
        self.a2 = float(kisa[1])
        self.a3 = float(kisa[2])
        self.b1 = float(kisa[3])
        self.b2 = float(kisa[4])
        self.b3 = float(kisa[5])
        self.g1 = float(kisa[6])


    def calculate_offset(self,q):
        az = math.radians(q.data[0])
        el = math.radians(q.data[1])

        cos_az = math.cos(az)
        sin_az = math.sin(az)
        cos_el = math.cos(el)
        sin_el = math.sin(el)
        pi = math.pi

        ## d_az[deg] , d_el[deg] ##
        d_az = self.a1*sin_el + self.a2 + self.a3*cos_el + self.b1*sin_az*sin_el - self.b2*cos_az*sin_el
        d_el = self.b1*cos_az + self.b2*sin_az + self.b3 + self.g1*el
        ### convert to encoder offset on the horizon ###
        d_az = d_az / cos_el

        ### apply the correction values  ###
        az2 = q.data[0] + d_az
        el2 = q.data[1] + d_el

        self.pub_az.publish(az2)
        self.pub_el.publish(el2)


if __name__ == "__main__":
    rospy.init_node(name)
    azel = refracted2apparent()
    rospy.spin()
