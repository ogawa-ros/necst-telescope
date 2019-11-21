#!/usr/bin/env python3

name = "constant_speed_azel_test"

import time
import rospy
import threading
import sys
import datetime
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
from astropy.time import Time
from astropy.coordinates import FK5
import astropy.units as u
from astropy.coordinates import EarthLocation
from astropy.coordinates import SkyCoord
from astropy.coordinates import AltAz
import astropy.constants
sys.path.append("/home/exito/ros/src/necst-core/scripts")
import core_controller

class constant_speed_azel_test(object):

    latitude = 35.940874
    longitude = 138.472153
    height = 1386
    frame = 'fk5'
    nobeyama = EarthLocation(lat = latitude*u.deg, lon = longitude*u.deg, height = height*u.m)
    el_cmd =''
    az_cmd =''

    press =  1000
    temp = -2
    humid = 0.8
    #obswl = 230 #GHz
    obswl = 600000 #GHz

    def __init__(self):
        self.start_az = float(input("Start az = "))
        self.end_az = float(input("End az = "))
        self.start_el = float(input("Start el = "))
        self.end_el = float(input("End el = "))
        self.logger = core_controller.logger()
        self.pub_real_azel = rospy.Publisher('/necst_telescope/coordinate/refracted_azel_cmd', Float64MultiArray, queue_size=1)


    def create_az(self):
        start_az = self.start_az
        end_az = self.end_az
        self.az_cmd = start_az
        speed_az = 360/(24*3600) #deg/s
        dt  = 0.01
        while not rospy.is_shutdown():
            if self.az_cmd >= end_az:
                break
            else:
                self.az_cmd += speed_az*dt
                time.sleep(dt)
            continue
        print("Finish to send AZ")

    def create_el(self):
        start_el = self.start_el
        end_el = self.end_el
        self.el_cmd = start_el
        speed_el = 360/(24*3600) #deg/s
        dt  = 0.01
        while not rospy.is_shutdown():
            if self.el_cmd >= end_el:
                break
            else:
                self.el_cmd += speed_el*dt
                time.sleep(dt)
            continue
        print("Finish to send EL")

    def publish_azel(self):
        while not rospy.is_shutdown():
            if self.el_cmd != '':                #obstime = altaz.obstime
                alt = self.el_cmd
                az = self.az_cmd
                array = Float64MultiArray()
                #array.data = [obstime, az, alt]
                array.data = [az, alt]
                self.pub_real_azel.publish(array)
                time.sleep(0.1)
            else:
                time.sleep(1)
            continue

    def measurement(self):
        date = datetime.datetime.today().strftime('%Y%m%d_%H%M%S')
        file_name = name + '/' + date + '.necstdb'
        print(file_name)
        logger = core_controller.logger()
        self.start_thread_az()
        self.start_thread_el()
        input("Enter to star measurement !!!")
        logger.start(file_name)
        time.sleep(60)
        logger.stop()

    def start_thread(self):
        th = threading.Thread(target=self.publish_azel)
        th.setDaemon(True)
        th.start()

    def start_thread_el(self):
        th = threading.Thread(target=self.create_el)
        th.setDaemon(True)
        th.start()

    def start_thread_az(self):
        th = threading.Thread(target=self.create_az)
        th.setDaemon(True)
        th.start()

if __name__ == "__main__":
    rospy.init_node(name)

    azel = constant_speed_azel_test()
    azel.start_thread()
    azel.measurement()
    print("Finish!!")
