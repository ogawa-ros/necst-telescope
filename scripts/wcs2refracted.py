#!/usr/bin/env python3

name = "wcs2refracted"

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


class wcs2refracted(object):

    latitude = 35.940874
    longitude = 138.472153
    height = 1386
    wcs_frame = 'fk5'
    nobeyama = EarthLocation(lat = latitude*u.deg, lon = longitude*u.deg, height = height*u.m)
    wcs =''
    wcs_li = []

    press =  1000
    temp =  0
    humid = 0.5
    obswl = 230 #GHz
    #obswl = 600000 #GHz

    def __init__(self):
        rospy.Subscriber('/necst/telescope/coordinate/wcs_cmd',Float64MultiArray,self.recieve_wcs)
        rospy.Subscriber('/necst/telescope/coordinate/wcs_frame_cmd',String,self.recieve_wcs_frame)
        rospy.Subscriber('/necst/telescope/coordinate/stop_cmd' ,Bool, self.recieve_stop_cmd)

        rospy.Subscriber('/necst/telescope/weather/pressure',Float64,self.recieve_pressure)
        rospy.Subscriber('/necst/telescope/weather/temperature',Float64,self.recieve_temprature)
        rospy.Subscriber('/necst/telescope/weather/humidity',Float64,self.recieve_humidity)
        rospy.Subscriber('/necst/telescope/obswl',Float64,self.recieve_obswl)

        self.pub_real_azel = rospy.Publisher('/necst/telescope/coordinate/refracted_azel_cmd', Float64MultiArray, queue_size=1)
        self.pub_wcs = rospy.Publisher('/necst/telescope/coordinate/wcs', Float64MultiArray, queue_size=1)
        self.init_flag  = True

    def recieve_wcs(self,q):
        self.wcs = q.data

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

    def recieve_stop_cmd(self,q):
        if q.data == True:
            self.wcs = ''
            self.init_flag  = True
        else:
            pass

    def convert_azel(self,dt):
        on_coord = SkyCoord(self.wcs[0]+self.wcs[2], self.wcs[1]+self.wcs[3],frame=self.wcs_frame, unit=(u.deg, u.deg))
        on_coord.location = self.nobeyama
        on_coord.pressure = self.press*u.hPa
        on_coord.temperature = self.temp*u.deg_C
        on_coord.relative_humidity = self.humid
        on_coord.obswl = (astropy.constants.c/(self.obswl*u.GHz)).to('micron')
        altaz_list = on_coord.transform_to(AltAz(obstime=Time.now()+TimeDelta(dt, format='sec')))
        return altaz_list

    def publish_azel(self):
        while not rospy.is_shutdown():
            if self.wcs != '':
                x = self.wcs[0]+self.wcs[2]
                y = self.wcs[1]+self.wcs[3]
                if self.init_flag == True:
                    for i in range(10):
                        altaz = self.convert_azel(dt=0.1*i)
                        obstime = altaz.obstime.to_value("unix")
                        alt = altaz.alt.deg
                        az = altaz.az.deg
                        array = Float64MultiArray()
                        array.data = [obstime, az, alt]
                        self.pub_real_azel.publish(array)

                        data = [obstime,x,y]
                        self.wcs_li.append(data)

                        time.sleep(0.0001)
                    self.init_flag = False

                else:
                    altaz = self.convert_azel(dt=1)
                    obstime = altaz.obstime.to_value("unix")
                    alt = altaz.alt.deg
                    az = altaz.az.deg
                    array = Float64MultiArray()
                    array.data = [obstime, az, alt]
                    self.pub_real_azel.publish(array)

                    data = [obstime,x,y]
                    self.wcs_li.append(data)

            else:
                time.sleep(0.001)
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
        th = threading.Thread(target=self.publish_azel)
        th.setDaemon(True)
        th.start()

        th2 = threading.Thread(target=self.wcs_pub)
        th2.setDaemon(True)
        th2.start()

if __name__ == "__main__":
    rospy.init_node(name)
    azel = wcs2refracted()
    azel.start_thread()
    rospy.spin()
