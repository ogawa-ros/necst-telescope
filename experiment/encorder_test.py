#!/usr/bin/env python3
"""
For test of telescope driving
"""
import sys
import ephem
import time
import numpy
import math
import os
import pylab
import datetime
import matplotlib.pyplot as plt
sys.path.append("/home/exito/ros/src/necst-telescope/scripts")
import telescope_controller
sys.path.append("/home/exito/ros/src/necst-core/scripts")
import core_controller
import rospy

name = "encorder_test"

rospy.init_node(name)

logger = core_controller.logger()
antenna = telescope_controller.antenna()

date = datetime.datetime.today().strftime('%Y%m%d_%H%M%S')
file_name = name + '/' + date + '.necstdb'
print(file_name)

az = antenna.get_az()
el = antenna.get_el()
print("current position : az="+str(az)+" el="+str(el))

antenna.move_azel(float(180),float(18))
antenna.tracking_check()
time.sleep(1)
antenna.move_azel(float(180),float(82))
logger.start(file_name)
antenna.tracking_check()
logger.stop()

pritn("finish !!")
