#!/usr/bin/env python3

import math
import rospy
import threading
from std_msgs.msg import Float64
from std_msgs.msg import Int64
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
    c1 = 0
    c2 = 0
    d1 = 0
    d2 = 0
    e1 = 0
    e2 = 0

    optobs = False

    def __init__(self):
        self.read_kisa()
        self.pub_az = rospy.Publisher("/necst/telescope/coordinate/apparent_az_cmd",Float64, queue_size=1)
        self.pub_el = rospy.Publisher("/necst/telescope/coordinate/apparent_el_cmd", Float64, queue_size=1)
        self.pub_cmd_num = rospy.Publisher("/necst/telescope/coordinate/apparent_cmd_num", Int64, queue_size=1)

        rospy.Subscriber('/necst/telescope/coordinate/refracted_azel_cmd', Float64MultiArray, self.recieve_azel)
        rospy.Subscriber('/necst/telescope/coordinate/stop_cmd' ,Bool, self.recieve_stop_cmd)

        rospy.Subscriber('/necst/telescope/coordinate/optobs', Bool, self.recieve_optobs)

    def recieve_azel(self, array):
        if array.data[0] > time.time():
            self.azel.append(array.data)
            self.azel.sort()
        else:
            pass

    def recieve_stop_cmd(self, q):
        self.azel = []

    def recieve_optobs(self, q):
        self.optobs = q.data

    def time_handler(self):
        while not rospy.is_shutdown():
            try:
                azel = self.azel.pop(0)
                self.pub_cmd_num.publish(len(self.azel))
            except:
                self.pub_cmd_num.publish(len(self.azel))
                time.sleep(0.001)
                continue

            while True:
                if azel[0] < time.time():
                    q = [azel[1],azel[2]] #[az,el]
                    self.calculate_kisa(q)
                    break

                elif azel[0] < time.time()
                    break
                else:
                    time.sleep(0.001)
                    continue
            #time.sleep(0.1)
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
        self.c1 = float(kisa[7])
        self.c2 = float(kisa[8])
        self.d1 = float(kisa[9])
        self.d2 = float(kisa[10])
        self.e1 = float(kisa[11])
        self.e2 = float(kisa[12])


    def calculate_kisa(self,azel):
        az = math.radians(azel[0])
        el = math.radians(azel[1])
        el2 = math.degrees(azel[1]/180*math.pi)

        cos_az = math.cos(az)
        sin_az = math.sin(az)
        cos_el = math.cos(el)
        sin_el = math.sin(el)
        cos_azel = math.cos(az-el)
        sin_azel = math.sin(az-el)
        pi = math.pi

        ## d_az[deg] , d_el[deg] ##
        d_az = self.a1*sin_el + self.a2 + self.a3*cos_el + self.b1*sin_az*sin_el - self.b2*cos_az*sin_el
        d_el = self.b1*cos_az + self.b2*sin_az + self.b3 + self.g1*el2

        if self.optobs == False:
            d_az = d_az + self.c1*sin_azel + self.c2*cos_azel + self.d1+ self.e1*cos_el - self.e2*sin_el
            d_el = d_el + self.c1*cos_azel - self.c2*sin_azel + self.d2+ self.e1*sin_el + self.e2*cos_el
        else:
            pass

        ### convert to encoder offset on the horizon ###
        d_az = d_az / cos_el

        ### apply the correction values  ###
        azaz = azel[0] + d_az
        elel = azel[1] + d_el

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
