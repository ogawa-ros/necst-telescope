#!/usr/bin/env python3

name = "wcs2refracted_raster_azel"

import time
import rospy
import threading
import numpy
import math
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
from std_msgs.msg import String

from astropy.time import Time
from astropy.time import TimeDelta
from astropy.coordinates import FK5
import astropy.units as u
from astropy.coordinates import EarthLocation
from astropy.coordinates import SkyCoord
from astropy.coordinates import AltAz
import astropy.constants
from astropy.utils.iers import conf


class wcs2refracted_raster_azel(object):

    latitude = 35.940874
    longitude = 138.472153
    height = 1386
    wcs_frame = 'fk5'
    nobeyama = EarthLocation(lat = latitude*u.deg, lon = longitude*u.deg, height = height*u.m)
    wcs =''

    press =  1000
    temp =  0
    humid = 0.5
    obswl = 230 #GHz
    #obswl = 600000 #GHz

    offset_li = []
    wcs_li = []

    def __init__(self):
        rospy.Subscriber('/necst/telescope/coordinate/wcs_raster_azel_cmd',Float64MultiArray,self.publish)
        rospy.Subscriber('/necst/telescope/coordinate/wcs_frame_cmd',String,self.recieve_wcs_frame)

        rospy.Subscriber('/necst/telescope/weather/pressure',Float64,self.recieve_pressure)
        rospy.Subscriber('/necst/telescope/weather/temperature',Float64,self.recieve_temprature)
        rospy.Subscriber('/necst/telescope/weather/humidity',Float64,self.recieve_humidity)
        rospy.Subscriber('/necst/telescope/obswl',Float64,self.recieve_obswl)
        rospy.Subscriber('/necst/telescope/tracking_check',Bool,self.track_check)


        self.pub_real_azel = rospy.Publisher('/necst/telescope/coordinate/refracted_azel_cmd', Float64MultiArray, queue_size=1)
        self.pub_raster_check = rospy.Publisher('/necst/telescope/coordinate/raster_check', Bool, queue_size=1)
        self.pub_offset = rospy.Publisher('/necst/telescope/coordinate/azel_offset', Float64MultiArray, queue_size=1)
        self.pub_stop_cmd = rospy.Publisher('/necst/telescope/coordinate/stop_cmd', Bool, queue_size=1)

    def recieve_wcs_frame(self,q):
        self.wcs_frame = q.data

    def recieve_pressure(self,q):
        self.press = q.data

    def recieve_temprature(self,q):
        self.temp = q.data

    def recieve_humidity(self,q):
        self.humid = q.data

    def recieve_obswl(self,q):
        self.obswl = q.data

    def track_check(self,q):
        self.tracking_check = q.data

    def publish(self,q):
        print("aaaaa")
        x = q.data[0]
        y = q.data[1]
        lx = q.data[2]
        ly = q.data[3]
        scan_t = q.data[4]

        length = (lx**2 + ly**2)**(1/2)
        dl = length/scan_t * 0.1
        dx = dl * lx/length
        dy = dl * ly/length
        start_x = -lx/2
        start_y = -ly/2
        num = int(length/dl)

        self.pub_raster_check.publish(True)

        #scan開始点に移動
        for i in range(10):
            t = 0.1 * i
            altaz = self.convert_azel(x,y,dt=t)
            obstime = altaz.obstime.to_value("unix")
            az  = altaz.az.deg  + start_x/numpy.cos(math.radians(altaz.alt.deg))
            alt = altaz.alt.deg + start_y
            array = Float64MultiArray()
            array.data = [obstime, az, alt]
            self.pub_real_azel.publish(array)
            time.sleep(0.001)
            print("first")

        print(0)
        while not self.tracking_check :
            altaz = self.convert_azel(x,y,dt=1)
            obstime = altaz.obstime.to_value("unix")
            az  = altaz.az.deg  + start_x/numpy.cos(math.radians(altaz.alt.deg))
            alt = altaz.alt.deg + start_y

            array = Float64MultiArray()
            array.data = [obstime, az, alt]
            print(array)
            self.pub_real_azel.publish(array)
            time.sleep(0.1)

        print(1)
        self.pub_stop_cmd.publish(True)
        time.sleep(1)

        #scan開始
        for i in range(num+1):
            print(i)
            offset_x = start_x + dx*i
            offset_y = start_y + dy*i

            altaz = self.convert_azel(x,y)

            obstime = altaz.obstime.to_value("unix")
            az  = altaz.az.deg  + offset_x/numpy.cos(math.radians(altaz.alt.deg))
            alt = altaz.alt.deg + offset_y

            array = Float64MultiArray()
            array.data = [obstime, az, alt]
            self.pub_real_azel.publish(array)

            self.offset_li.append([obstime, offset_x, offset_y])

            if i == num:
                last_obstime = obstime

        while last_obstime > time.time():
            time.sleep(0.1)
            continue
        self.pub_raster_check.publish(False)
        pass

    def convert_azel(self,x,y,dt=0):
        on_coord = SkyCoord(x, y,frame=self.wcs_frame, unit=(u.deg, u.deg))
        on_coord.location = self.nobeyama
        on_coord.pressure = self.press*u.hPa
        on_coord.temperature = self.temp*u.deg_C
        on_coord.relative_humidity = self.humid
        on_coord.obswl = (astropy.constants.c/(self.obswl*u.GHz)).to('micron')
        altaz_list = on_coord.transform_to(AltAz(obstime=Time.now()+TimeDelta(dt,format="sec")))
        return altaz_list

    def offset_pub(self):
        while not rospy.is_shutdown():
            #print(len(self.offset_li))
            try:
                offset = self.offset_li.pop(0)
            except:
                time.sleep(0.0001)
                continue

            while True:
                if offset[0] < time.time():
                    q = Float64MultiArray()
                    q.data = [offset[0],offset[1],offset[2]] #[time, offset_az,offset_el]
                    self.pub_offset.publish(q)
                    break
                else:
                    time.sleep(0.0001)
                    continue

            continue


    def start_thread(self):
        th = threading.Thread(target=self.offset_pub)
        th.setDaemon(True)
        th.start()


if __name__ == "__main__":
    rospy.init_node(name)
    wcs = wcs2refracted_raster_azel()
    wcs.start_thread()
    rospy.spin()
