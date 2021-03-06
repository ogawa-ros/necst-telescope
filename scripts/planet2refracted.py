#!/usr/bin/env python3

name = "planet2refracted"

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
import astropy.constants

class planet2refracted(object):

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

    def __init__(self):
        rospy.Subscriber('/necst/telescope/coordinate/planet_cmd',Float64MultiArray,self.recieve_planet)
        rospy.Subscriber('/necst/telescope/weather/pressure',Float64,self.recieve_pressure)
        rospy.Subscriber('/necst/telescope/weather/temperature',Float64,self.recieve_temprature)
        rospy.Subscriber('/necst/telescope/weather/humidity',Float64,self.recieve_humidity)
        rospy.Subscriber('/necst/telescope/coordinate/stop_cmd' ,Bool, self.recieve_stop_cmd)
        rospy.Subscriber('/necst/telescope/obswl',Float64,self.recieve_obswl)

        self.pub_real_azel = rospy.Publisher('/necst/telescope/coordinate/refracted_azel_cmd', Float64MultiArray, queue_size=1)

        self.init_flag  = True

    def recieve_planet(self,q):
        self.planet = [q.data[0],q.data[1],q.data[2]]

        if self.planet[0]==0 : self.planet[0] = "sun"
        if self.planet[0]==1 : self.planet[0] = "moon"
        if self.planet[0]==2 : self.planet[0] = "mercury"
        if self.planet[0]==3 : self.planet[0] = "venus"
        if self.planet[0]==4 : self.planet[0] = "mars"
        if self.planet[0]==5 : self.planet[0] = "jupiter"
        if self.planet[0]==6 : self.planet[0] = "saturn"
        if self.planet[0]==7 : self.planet[0] = "uranus"
        if self.planet[0]==8 : self.planet[0] = "neptune"


    def recieve_pressure(self,q):
        self.press = q.data

    def recieve_temprature(self,q):
        self.temp = q.data

    def recieve_humidity(self,q):
        self.humid = q.data

    def recieve_stop_cmd(self,q):
        if q.data == True:
            self.planet = ""
            self.init_flag  = True
        else:
            pass

    def recieve_obswl(self,q):
        self.obswl = q.data

    def convert_azel(self,dt):
        target = self.planet
        on_coord = astropy.coordinates.get_body(location=self.nobeyama,time=Time.now()+TimeDelta(dt, format='sec'),body=target[0])
        on_coord.location = self.nobeyama
        on_coord.pressure = self.press*u.hPa
        on_coord.temperature = self.temp*u.deg_C
        on_coord.relative_humidity = self.humid
        on_coord.obswl = (astropy.constants.c/(self.obswl*u.GHz)).to('micron')
        off_x = target[1]
        off_y = target[2]
        return on_coord.altaz,off_x,off_y

    def publish_azel(self):
        while not rospy.is_shutdown():
            if self.planet != "":
                if self.init_flag == True:
                    for i in range(11):
                        altaz,off_x,off_y = self.convert_azel(dt=0.1*i)
                        obstime = altaz.obstime.to_value("unix")
                        alt = altaz.alt.deg + off_y
                        az  = altaz.az.deg  + off_x
                        array = Float64MultiArray()
                        array.data = [obstime, az, alt]
                        self.pub_real_azel.publish(array)
                        time.sleep(0.0001)
                    self.init_flag  = False

                else:
                    altaz,off_x,off_y = self.convert_azel(dt=1)
                    obstime = altaz.obstime.to_value("unix")
                    alt = altaz.alt.deg + off_y
                    az  = altaz.az.deg  + off_x
                    array = Float64MultiArray()
                    array.data = [obstime, az, alt]
                    self.pub_real_azel.publish(array)
                    time.sleep(0.1)

            else:
                time.sleep(0.01)
            continue

    def start_thread(self):
        th = threading.Thread(target=self.publish_azel)
        th.setDaemon(True)
        th.start()

if __name__ == "__main__":
    rospy.init_node(name)
    azel = planet2refracted()
    azel.start_thread()
    rospy.spin()
