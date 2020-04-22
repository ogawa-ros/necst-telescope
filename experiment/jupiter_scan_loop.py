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

planet = "jupiter"
scan_direction = 1 #1or-1
_lx = 1 #deg
_ly = 1 #deg
scan_t = 60 

num = 5

name = "jupiter_scan_loop"
rospy.init_node(name)

logger = core_controller.logger()
antenna = telescope_controller.antenna()
load = controller_1p85m2019.load()

status = rospy.Publisher('/'+name+'/scan_status', std_msgs.msg.String, queue_size=1)
direct = rospy.Publisher('/'+name+'/scan_direction', std_msgs.msg.Float64, queue_size=1)

date = datetime.datetime.today().strftime('%Y%m%d_%H%M%S')
file_name = name + '/' + date + '.necstdb'
lx = _lx*scan_direction
ly = _ly*scan_direction
print(file_name)

time.sleep(1)

print('START JUPITER SCAN')
print("===========================")
print("length of x : "+str(lx)+"[deg]")
print("length of y : "+str(ly)+"[deg]")
print("scan time : "+str(scan_t)+"[s]")
print("scan num : "+str(num))
print("===========================")

logger.start(file_name)
time.sleep(1)
direct.publish(scan_direction)

antenna.move_planet(planet,-lx/2,0)
antenna.tracking_check()

status.publish("{0:4}".format("offs"))
time.sleep(5)
status.publish("{0:4}".format("offe"))

for i in range(num):

    load.move_hot()
    time.sleep(5)
    load.move_sky()
    time.sleep(5)

    antenna.move_planet(planet,-lx/2,0)
    antenna.tracking_check()

    print("{0:2}{1:2}".format("az",i+1))
    status.publish("{0:2}{1:2}".format("az",i+1))
    antenna.move_raster_planet(planet,lx=lx,ly=0 ,scan_t=scan_t,l_unit="deg")
    time.sleep(5)


    antenna.move_planet(planet,0,-ly/2)
    antenna.tracking_check()

    print("{0:2}{1:2}".format("el",i+1))
    status.publish("{0:2}{1:2}".format("el",i+1))
    antenna.move_raster_planet(planet,lx=0 ,ly=ly,scan_t=scan_t,l_unit="deg")
    time.sleep(5)

logger.stop()

antenna.finalize()
