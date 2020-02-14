#!/usr/bin/env python3

name = "planet2refracted_raster"

import time
import rospy
import threading
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from std_msgs.msg import Bool

from astropy.time import Time
from astropy.time import TimeDelta
from astropy.coordinates import FK5
import astropy.units as u
from astropy.coordinates import EarthLocation
from astropy.coordinates import SkyCoord
from astropy.coordinates import AltAz
from astropy.coordinates import solar_system_ephemeris
import astropy.constants

class planet2refracted_raster(object):

    latitude = 35.940874
    longitude = 138.472153
    height = 1386
    frame = 'fk5'
    nobeyama = EarthLocation(lat = latitude*u.deg, lon = longitude*u.deg, height = height*u.m)
    planet = ""

    press =  1000
    temp = 10
    humid = 0.5
    obswl = 230 #GHz

    offset_li = []

    def __init__(self):
        rospy.Subscriber('/necst/telescope/coordinate/planet_raster_cmd',Float64MultiArray,self.publish)
        rospy.Subscriber('/necst/telescope/weather/pressure',Float64,self.recieve_pressure)
        rospy.Subscriber('/necst/telescope/weather/temperature',Float64,self.recieve_temprature)
        rospy.Subscriber('/necst/telescope/weather/humidity',Float64,self.recieve_humidity)
        rospy.Subscriber('/necst/telescope/obswl',Float64,self.recieve_obswl)

        self.pub_real_azel = rospy.Publisher('/necst/telescope/coordinate/refracted_azel_cmd', Float64MultiArray, queue_size=10)
        self.pub_offset = rospy.Publisher('/necst/telescope/coordinate/planet_offset', Float64MultiArray, queue_size=10)
        self.pub_raster_check = rospy.Publisher('/necst/telescope/coordinate/raster_check', Bool, queue_size=1)

    def recieve_pressure(self,q):
        self.press = q.data

    def recieve_temprature(self,q):
        self.temp = q.data

    def recieve_humidity(self,q):
        self.humid = q.data

    def recieve_obswl(self,q):
        self.obswl = q.data

    def convert_azel(self,planet,times):
        on_coord = astropy.coordinates.get_body(location=self.nobeyama,time=times,body=planet)
        #solar_system_ephemeris.set('de432s') #between 1950-2050
        on_coord.location = self.nobeyama
        on_coord.pressure = self.press*u.hPa
        on_coord.temperature = self.temp*u.deg_C
        on_coord.relative_humidity = self.humid
        on_coord.obswl = (astropy.constants.c/(self.obswl*u.GHz)).to('micron')
        return on_coord.altaz

    def publish(self,q):

        if q.data[0]==0 : planet = "sun"
        if q.data[0]==1 : planet = "moon"
        if q.data[0]==2 : planet = "mercury"
        if q.data[0]==3 : planet = "venus"
        if q.data[0]==4 : planet = "mars"
        if q.data[0]==5 : planet = "jupiter"
        if q.data[0]==6 : planet = "saturn"
        if q.data[0]==7 : planet = "uranus"
        if q.data[0]==8 : planet = "neptune"

        lx = q.data[1]
        ly = q.data[2]
        scan_t = q.data[3]
        print(planet,lx,ly,scan_t)

        length = (lx**2 + ly**2)**(1/2)
        dl = length/scan_t * 0.1
        dx = dl * lx/length
        dy = dl * ly/length
        start_x = -lx/2
        start_y = -ly/2

        num = int(length/dl)
        print(1)
        t0 = Time.now()
        times = t0 + numpy.linspace(0, scan_t, num+1)*u.s
        altaz = self.convert_azel(planet,times)
        print(2)
        self.pub_raster_check.publish(True)

        for i in range(num+1):
            offset_x = start_x + dx*i
            offset_y = start_y + dy*i

            obstime = altaz[i].obstime.to_value("unix")
            az  = altaz[i].az.deg  + offset_x
            alt = altaz[i].alt.deg + offset_y

            array = Float64MultiArray()
            array.data = [obstime, az, alt]
            self.pub_real_azel.publish(array)

            self.offset_li.append([obstime,offset_x,offset_y])
            time.sleep(0.001)
            print(3)
            if i == num:
                print(4)
                last_obstime = obstime

        print(5)
        while last_obstime > Time.now():
            print(6)
            time.sleep(0.1)
            continue
        print(7)
        self.pub_raster_check.publish(False)
        pass

    def offfset_pub(self):
        while not rospy.is_shutdown():
            #print(len(self.offset_li))
            try:
                offset = self.offset_li.pop(0)
            except:
                time.sleep(0.00001)
                continue

            while True:
                if offset[0] < time.time():
                    q = Float64MultiArray()
                    q.data = [offset[1],offset[2]] #[offset_az,offset_el]
                    self.pub_offset.publish(q)
                    break
                else:
                    time.sleep(0.0001)
                    continue

            continue

    def start_thread(self):
        th = threading.Thread(target=self.offfset_pub)
        th.setDaemon(True)
        th.start()

if __name__ == "__main__":
    rospy.init_node(name)
    azel = planet2refracted_raster()
    azel.start_thread()
    rospy.spin()
