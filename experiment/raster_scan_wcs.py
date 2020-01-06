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
import rospy

x = 0
y = 0
lx = 1 #deg
ly = 0 #deg
scan_t = 60
frame = "fk5"

name = "raster_scan_" + planet
rospy.init_node(name)

logger = core_controller.logger()
antenna = telescope_controller.antenna()

date = datetime.datetime.today().strftime('%Y%m%d_%H%M%S')
file_name = name + '/' + date + '.necstdb'
print(file_name)

logger.start(file_name)

antenna.move_raster_wcs(x,y,lx,ly,scan_t,l_unit="deg",frame=frame)
logger.stop()
