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

object = "IRC+10216"

scan_direction = 1 #1or-1
on_x = 146.989193 #deg
on_y = 13.278768 #deg
on_frame = "fk5"

off_x = 146.562 #deg
off_y = 13.5125 #deg
off_frame = "fk4" #B1950

off_integ = 3
hot_integ = 3

_lx = 2 #deg
_ly = 2 #deg
scan_t = 60

num = 3

name = "wcs_scan_loop"
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

print('START CROSS SCAN')
print("===========================")
print("length of x : "+str(lx)+"[deg]")
print("length of y : "+str(ly)+"[deg]")
print("scan time : "+str(scan_t)+"[s]")
print("scan num : "+str(num))
print("===========================")

logger.start(file_name)
time.sleep(1)
direct.publish(scan_direction)

antenna.move_wcs(off_x,off_y,frame=off_frame)
antenna.tracking_check()

status.publish("{0:9}".format("off_start"))
time.sleep(off_integ)
status.publish("{0:9}".format("off_end"))

for i in range(num):
    load.move_hot()
    load.check_hot()
    status.publish("{0:9}".format("hot_start"))
    time.sleep(hot_integ)
    status.publish("{0:9}".format("hot_end"))

    load.move_sky()
    load.check_sky()

    print("{0:9}".format("az_start"))
    status.publish("{0:9}".format("az_start"))
    antenna.move_raster_wcs_azel(on_x,on_y,lx=lx,ly=0 ,scan_t=scan_t, frame=on_frame)
    status.publish("{0:9}".format("az_end"))
    print("{0:9}".format("az_end"))
    time.sleep(1)

    print("{0:9}".format("el_start"))
    status.publish("{0:9}".format("el_start"))
    antenna.move_raster_wcs_azel(on_x,on_y,lx=0 ,ly=ly,scan_t=scan_t, frame=on_frame)
    status.publish("{0:9}".format("el_end"))
    print("{0:9}".format("el_end"))
    time.sleep(1)

logger.stop()

antenna.finalize()
