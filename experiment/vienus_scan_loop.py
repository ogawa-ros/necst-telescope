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

planet = "venus"
lx = 2 #deg
ly = 2 #deg
scan_t = 60

num = 5

name = "venus_scan_loop"
rospy.init_node(name)

logger = core_controller.logger()
antenna = telescope_controller.antenna()
load = controller_1p85m2019.load()

status = rospy.Publisher('/'+name+'/scan_status', std_msgs.msg.String, queue_size=1)

date = datetime.datetime.today().strftime('%Y%m%d_%H%M%S')
file_name = name + '/' + date + '.necstdb'
print(file_name)

time.sleep(1)

print('START VENUS SCAN')
print("===========================")
print("length of x : "+str(lx)+"[deg]")
print("length of y : "+str(ly)+"[deg]")
print("scan time : "+str(scan_t)+"[s]")
print("scan num : "+str(num))
print("===========================")

logger.start(file_name)

for i in range(num):
    antenna.move_planet(planet,-lx/2,0)
    load.move_hot()
    time.sleep(5)
    load.move_sky()
    time.sleep(5)
    antenna.tracking_check()

    print("{0:2}{1:2}".format("az",i+1))
    status.publish("{0:2}{1:2}".format("az",i+1))
    antenna.move_raster_planet(planet,lx=lx,ly=0 ,scan_t=scan_t,l_unit="deg")
    time.sleep(1)

    print("{0:2}{1:2}".format("el",i+1))
    status.publish("{0:2}{1:2}".format("el",i+1))
    antenna.move_raster_planet(planet,lx=0 ,ly=ly,scan_t=scan_t,l_unit="deg")

logger.stop()

antenna.finalize()
