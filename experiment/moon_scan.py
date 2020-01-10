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
import std_msgs

planet = "moon"
lx = 1.5 #deg
ly = 1.5 #deg
scan_t = 45

name = "moon_scan"
rospy.init_node(name)

logger = core_controller.logger()
antenna = telescope_controller.antenna()
load = controller_1p85m2019.load()

status = rospy.Publisher('/'+name+'/scan_status', std_msgs.msg.String, queue_size=1)

date = datetime.datetime.today().strftime('%Y%m%d_%H%M%S')
file_name = name + '/' + date + '.necstdb'
print(file_name)

antenna.move_planet(planet,-lx/2,-ly/2) #deg
antenna.tracking_check()
time.sleep(1)

print('START MOON SCAN')

logger.start(file_name)

load.move_hot()
time.sleep(5)
load.move_sky()
time.sleep(5)

status.publish("{0:2}".format("az"))
antenna.move_raster_planet(planet,lx=lx,ly=0 ,scan_t=scan_t,l_unit="deg")
time.sleep(1)

status.publish("{0:2}".format("el"))
antenna.move_raster_planet(planet,lx=0 ,ly=ly,scan_t=scan_t,l_unit="deg")

logger.stop()

antenna.finalize()
