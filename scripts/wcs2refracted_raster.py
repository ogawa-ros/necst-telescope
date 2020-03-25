#!/usr/bin/env python3

name = "wcs2refracted_raster"

import time
import rospy
import threading
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


class wcs2refracted_raster(object):

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
        rospy.Subscriber('/necst/telescope/coordinate/wcs_raster_cmd',Float64MultiArray,self.publish)
        rospy.Subscriber('/necst/telescope/coordinate/wcs_frame_cmd',String,self.recieve_wcs_frame)

        rospy.Subscriber('/necst/telescope/weather/pressure',Float64,self.recieve_pressure)
        rospy.Subscriber('/necst/telescope/weather/temperature',Float64,self.recieve_temprature)
        rospy.Subscriber('/necst/telescope/weather/humidity',Float64,self.recieve_humidity)
        rospy.Subscriber('/necst/telescope/obswl',Float64,self.recieve_obswl)

        self.pub_real_azel = rospy.Publisher('/necst/telescope/coordinate/refracted_azel_cmd', Float64MultiArray, queue_size=1)
        self.pub_raster_check = rospy.Publisher('/necst/telescope/coordinate/raster_check', Bool, queue_size=1)
        self.pub_offset = rospy.Publisher('/necst/telescope/coordinate/wcs_offset', Float64MultiArray, queue_size=1)
        self.pub_wcs = rospy.Publisher('/necst/telescope/coordinate/wcs', Float64MultiArray, queue_size=1)


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

    def publish(self,q):
        x = q.data[0]
        y = q.data[1]
        lx = q.data[2]
        ly = q.data[3]
        scan_t = q.data[4]
        start_offset_px = q.data[5]
        start_offset_py = q.data[6]

        length = (lx**2 + ly**2)**(1/2)
        dl = length/scan_t * 0.1
        dx = dl * lx/length
        dy = dl * ly/length
        x = x - start_offset_px
        y = y - start_offset_py
        num = int(length/dl)

        self.pub_raster_check.publish(True)

        for i in range(num+1):
            offset_x = dx*i
            offset_y = dy*i
            dt = 0.1*i
            altaz = self.convert_azel(x,y,offset_x,offset_y,dt)
            obstime = altaz[i].obstime.to_value("unix")

            array = Float64MultiArray()
            array.data = [obstime, altaz.az.deg, altaz.alt.deg]
            self.pub_real_azel.publish(array)

            self.offset_li.append([obstime, offset_x, offset_y])
            self.wcs_li.append([obstime, x+offset_x, y+offset_y])

            if i == num:
                last_obstime = obstime

        while obstime > time.time():
            time.sleep(0.1)
            continue
        self.pub_raster_check.publish(False)
        pass

    def convert_azel(self,x,y,offset_x,offset_y,times):
        on_coord = SkyCoord(x+offset_x, y+offset_y,frame=self.wcs_frame, unit=(u.deg, u.deg))
        on_coord.location = self.nobeyama
        on_coord.pressure = self.press*u.hPa
        on_coord.temperature = self.temp*u.deg_C
        on_coord.relative_humidity = self.humid
        on_coord.obswl = (astropy.constants.c/(self.obswl*u.GHz)).to('micron')
        altaz_list = on_coord.transform_to(AltAz(obstime=Time.now()+TimeDelta(dt, format='sec')))
        return altaz_list

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
                    q.data = [offset[0],offset[1],offset[2]] #[time, offset_az,offset_el]
                    self.pub_offset.publish(q)
                    break
                else:
                    time.sleep(0.0001)
                    continue

            continue

    def wcs_pub(self):
        while not rospy.is_shutdown():
            #print(len(self.offset_li))
            try:
                wcs = self.wcs_li.pop(0)
            except:
                time.sleep(0.00001)
                continue

            while True:
                if wcs[0] < time.time():
                    q = Float64MultiArray()
                    q.data = [wcs[0],wcs[1],wcs[2]] #[time,x,y]
                    self.pub_wcs.publish(q)
                    break
                else:
                    time.sleep(0.0001)
                    continue

            continue

    def start_thread(self):
        th = threading.Thread(target=self.offfset_pub)
        th.setDaemon(True)
        th.start()

        th2 = threading.Thread(target=self.wcs_pub)
        th2.setDaemon(True)
        th2.start()


if __name__ == "__main__":
    rospy.init_node(name)
    wcs = wcs2refracted_raster()
    wcs.start_thread()
    rospy.spin()
