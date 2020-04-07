#!/usr/bin/env python3

import sys
import time
import numpy
import math
import os
import datetime
sys.path.append("/home/exito/ros/src/necst-telescope/scripts")
import telescope_controller
sys.path.append("/home/exito/ros/src/necst-core/scripts")
import core_controller
sys.path.append("/home/exito/ros/src/necst-1p85m2019/scripts")
import controller_1p85m2019
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String


###############parameter###################


name = "otf"
param = {}

#IRC+10216
#param["on_x"] = (9 + 45/60 + 14/3600)*15
#param["on_y"] = 13 + 30/60 + 40/3600

'13h49m01s -28d22m03s'
param["on_x"] = (5 + 35/60 + 14.46/3600)*15
param["on_y"] = -(5 + 22/60 + 29.6/3600)

param["on_frame"] = "fk5"

param["num_x"] = 50
param["num_y"] = 50
param["delta_x"] = 1/60
param["delta_y"] = 1/60
param["delta_t"] = 0.3

param["ramp"] = 1

param["off_x"] = 82.55910596
param["off_y"] = -5.66845794
param["off_frame"] = "fk5"
param["off_integ"] = 1

param["hot_time"] = 10
param["hot_interval"] = 10

param["direction"] = "H"

param["target"] = "test"


###################START OBSERVATION##########################

class otf_observation(object):

    last_timestamp = 0.
    interval = 10

    def __init__(self):
        self.logger = core_controller.logger()
        self.antenna = telescope_controller.antenna()
        self.load = controller_1p85m2019.load()

        self.obsmode = rospy.Publisher('/otf/obsmode', String, queue_size=1)
        self.target = rospy.Publisher('/otf/target', String, queue_size=1)

    def hot_obs(self,hot_time):
        self.load.move_hot()
        time.sleep(5)
        self.obsmode.publish("{0:9}".format('hot start'))
        time.sleep(hot_time)
        self.obsmode.publish("{0:9}".format('hot end'))
        self.load.move_sky()
        time.sleep(5)
        pass

    def off_obs(self,off_x,off_y,off_frame,off_integ):
        self.antenna.move_wcs(off_x,off_y,frame=off_frame)
        self.antenna.tracking_check()
        self.obsmode.publish("{0:9}".format('off start'))
        time.sleep(off_integ)
        self.obsmode.publish("{0:9}".format('off end'))
        time.sleep(1)
        pass

    def timer_regist(self,t):
        self.interval = t*60 #min->sec
        self.regist_time = time.time()
        pass

    def timer_check(self):
        now = time.time()
        if self.interval <= (now - self.regist_time):
            self.last_timestamp = now
            return True
        return False

    def start(self,param):
        name = "otf_test"
        date = datetime.datetime.today().strftime('%Y%m%d_%H%M%S')
        file_name = name + '/' + date + '.necstdb'
        print(file_name)

        hot_time = param["hot_time"]
        hot_interval = param["hot_interval"]

        if param["direction"] == "H":
            total_scan = param["num_y"]
            x = param["on_x"]
            y = param["on_y"]
            dx = param["delta_x"]
            dy = param["delta_y"]
            frame = param["on_frame"]
            ramp = param["ramp"]
            num = param["num_x"]
            dt = param["delta_t"]
            off_x = param["off_x"]
            off_y = param["off_y"]
            off_frame = param["off_frame"]
            off_integ = param["off_integ"]

        elif param["direction"] == "V":
            total_scan = param["num_x"]

        self.logger.start(file_name)
        print("first hot")

        self.hot_obs(hot_time)
        self.timer_regist(hot_interval)

        for scan_num in range(total_scan):
            #################HOT##############
            if self.timer_check():
                print("hot")
                self.antenna.move_wcs(off_x,off_y,frame=off_frame)
                self.hot_obs(hot_time)
                self.timer_regist(hot_interval)
            else:
                pass

            #################OFF##############
            print("off")

            self.off_obs(off_x,off_y,off_frame,off_integ)

            #################ON##############
            if param["direction"] == "H":
                _lx = dx * num
                lx = _lx + dx/dt*ramp
                ly = 0
                sx = x - lx/2 - dx/dt*ramp
                sy = y + dy*scan_num
                scan_t = dt*num + ramp

            elif param["direction"] == "V":
                pass

            self.obsmode.publish("{0:9}".format('on start'))
            print("scan "+str(scan_num))
            self.antenna.move_raster_wcs(sx,sy,lx,ly,scan_t,l_unit="deg",frame=frame)
            self.obsmode.publish("{0:9}".format('on finish'))

        self.antenna.finalize()
        self.logger.stop()

        return

if __name__ == "__main__":
    rospy.init_node(name)
    otf = otf_observation()
    otf.start(param)
