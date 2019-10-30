#!/usr/bin/env python3

name = "wcs2refracted"

import time
import rospy
import threading
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray

from astropy.time import Time
from astropy.coordinates import FK5
import astropy.units as u
from astropy.coordinates import EarthLocation
from astropy.coordinates import SkyCoord
from astropy.coordinates import AltAz
import astropy.constants
from astropy.utils.iers import conf


class wcs2refracted(object):

    latitude = 35.940874
    longitude = 138.472153
    height = 1386
    frame = 'fk5'
    nobeyama = EarthLocation(lat = latitude*u.deg, lon = longitude*u.deg, height = height*u.m)
    wcs =''

    press =  1000
    temp = 15
    humid = 0.2
    obswl = 230 #GHz

    def __init__(self):
        conf.auto_max_age = None
        rospy.Subscriber('/necst_telescope/coordinate/wcs_cmd',Float64MultiArray,self.recieve_wcs)
        rospy.Subscriber('/necst_telescope/weather/pressure',Float64,self.recieve_pressure)
        rospy.Subscriber('/necst_telescope/weather/temperature',Float64,self.recieve_temprature)
        rospy.Subscriber('/necst_telescope/weather/humidity',Float64,self.recieve_humidity)

        self.pub_real_azel = rospy.Publisher('/necst_telescope/coordinate/refracted_azel_cmd', Float64MultiArray, queue_size=1)

    def recieve_wcs(self,q):
        self.wcs = q.data

    def recieve_pressure(self,q):
        self.press = q.data

    def recieve_temprature(self,q):
        self.temp = q.data

    def recieve_humidity(self,q):
        self.humid = q.data


    def convert_azel(self):
        on_coord = SkyCoord(self.wcs[0], self.wcs[1],frame=frame, unit=(u.deg, u.deg))
        on_coord.location = self.nobeyama
        on_coord.pressure = self.press*u.hPa
        on_coord.temperature = self.temp*u.deg_C
        on_coord.relative_humidity = self.humid
        on_coord.obswl = (astropy.constants.c/(self.obswl*u.GHz)).to('micron')
        altaz_list = on_coord.transform_to(AltAz(obstime=Time.now()))
        return altaz_list

    def publish_azel(self):
        while not rospy.is_shutdown():
            if self.wcs != '':
                altaz = self.convert_azel()
                #obstime = altaz.obstime
                alt = altaz.alt.deg
                az = altaz.az.deg
                array = Float64MultiArray()
                #array.data = [obstime, az, alt]
                array.data = [az, alt]
                self.pub_real_azel.publish(array)
            else:
                pass
            continue

    def start_thread(self):
        th = threading.Thread(target=self.publish_azel)
        th.setDaemon(True)
        th.start()

if __name__ == "__main__":
    rospy.init_node(name)
    azel = wcs2refracted()
    azel.start_thread()
    rospy.spin()
