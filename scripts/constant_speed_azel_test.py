#!/usr/bin/env python3

name = "constant_speed_azel_test"

import time
import rospy
import threading
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

class constant_speed_azel_test(object):

    latitude = 35.940874
    longitude = 138.472153
    height = 1386
    frame = 'fk5'
    nobeyama = EarthLocation(lat = latitude*u.deg, lon = longitude*u.deg, height = height*u.m)
    el_cmd =''

    press =  1000
    temp = -2
    humid = 0.8
    #obswl = 230 #GHz
    obswl = 600000 #GHz

    def __init__(self):
        self.pub_real_azel = rospy.Publisher('/necst_telescope/coordinate/refracted_azel_cmd', Float64MultiArray, queue_size=1)


    def convert_refracted_azel(self):
        altaz = AltAz(location=nobeyama,
                      pressure=press*u.hPa,
                      temperature=temp*u.deg_C,
                      relative_humidity=humid,
                      obswl=(astropy.constants.c/(obswl*u.GHz)).to('micron')
                      )
        on_coord = SkyCoord(self.az_cmd, self.el_cmd,frame=altaz, unit=(u.deg, u.deg))
        altaz = on_coord.altaz
        return altaz

    def create_az(self,start_az,end_az):
        start_az = 45
        end_az = 80
        speed_az = 360/(24*3600) #deg/s
        dt  = 0.01
        while True:
            if self.az_cmd >= end_az:
                break
            else:
                self.az_cmd = start_az + speed_az*dt
                time.sleep(dt)
            continue

    def create_el(self,start_el,end_el):
        speed_el = 360/(24*3600) #deg/s
        dt  = 0.01
        while True:
            if self.el_cmd >= end_el:
                break
            else:
                self.el_cmd = start_el + speed_el*dt
                time.sleep(dt)
            continue

    def publish_azel(self):
        while not rospy.is_shutdown():
            if self.el_cmd != '':
                altaz = self.convert_refracted_azel()
                #obstime = altaz.obstime
                alt = altaz.alt.deg
                az = altaz.az.deg
                array = Float64MultiArray()
                #array.data = [obstime, az, alt]
                array.data = [az, alt]
                self.pub_real_azel.publish(array)
                time.sleep(0.1)
            else:
                time.sleep(1)
            continue

    def start_thread(self):
        th = threading.Thread(target=self.publish_azel)
        th.setDaemon(True)
        th.start()

if __name__ == "__main__":
    rospy.init_node(name)
    azel = constant_speed_azel_test()
    azel.start_thread()
    azel.create_az(start_az = 150,end_az = 210)
    azel.create_el(start_el = 25,end_el = 80)
    rospy.spin()
