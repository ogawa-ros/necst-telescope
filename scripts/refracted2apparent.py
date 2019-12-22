#!/usr/bin/env python3

import math
import rospy
import threading
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
import time

name = "refracted2apparent"
kisa_path = "/home/exito/ros/src/necst-1p85m2019/lib/kisa.dat"

class refracted2apparent(object):
    azel = []
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
        rospy.Subscriber('/necst/telescope/coordinate/refracted_azel_cmd', Float64MultiArray, self.recieve_azel)
        rospy.Subscriber('/necst/telescope/coordinate/stop_refracted_cmd' ,Bool, self.recieve_stop_cmd)


    def recieve_azel(self, array):
        self.azel.append(array.data)
        self.azel.sort()

    def recieve_stop_cmd(self, q):
        self.azel = []

    def time_handler(self):
        while not rospy.is_shutdown():
            try:
                azel = self.azel.pop(0)
                print(azel)
            except:
                continue
            print(azel[0])
            print(time.time())
            while True:
                if azel[0] < time.time():
                    q = [azel[1],azel[2]] #[az,el]
                    self.calculate_offset(q)
                    break
                else:
                    time.sleep(0.0001)
                    continue

            continue

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
        el2 = math.degrees(q.data[1]/180*math.pi)

        cos_az = math.cos(az)
        sin_az = math.sin(az)
        cos_el = math.cos(el)
        sin_el = math.sin(el)
        pi = math.pi

        ## d_az[deg] , d_el[deg] ##
        d_az = self.a1*sin_el + self.a2 + self.a3*cos_el + self.b1*sin_az*sin_el - self.b2*cos_az*sin_el
        d_el = self.b1*cos_az + self.b2*sin_az + self.b3 + self.g1*el2
        ### convert to encoder offset on the horizon ###
        d_az = d_az / cos_el

        ### apply the correction values  ###
        azaz = q.data[0] + d_az
        elel = q.data[1] + d_el

        self.pub_az.publish(azaz)
        self.pub_el.publish(elel)

    def start_thread(self):
        th = threading.Thread(target=self.time_handler)
        th.setDaemon(True)
        th.start()

if __name__ == "__main__":
    rospy.init_node(name)
    azel = refracted2apparent()
    azel.start_thread()
    rospy.spin()
